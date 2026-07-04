"""
analyze_sessions.py
Batch analyse all participant session folders and produce:
    analysis/per_participant.csv
    analysis/condition_summary.csv
    analysis/stats_tests.txt
    analysis/fig_distraction_profile.png
    analysis/fig_warning_rates.png
    analysis/fig_perceived_duration.png
    analysis/summary_report.md

Usage:
    python analyze_sessions.py
    python analyze_sessions.py --logs mylogs/
    python analyze_sessions.py --exclude pilot P01
"""

import argparse
import json
from pathlib import Path
from collections import defaultdict

import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
from scipy import stats
from scipy.stats import shapiro, ttest_ind, mannwhitneyu


T_WINDOW_S = 420.0
LONG_WARN_THRESHOLD_S = 10.0
ALPHA = 0.05

PRIMARY_OUTCOMES = [
    "pct_focused",
    "pct_warning",
    "total_warning_s",
    "n_warnings",
    "mean_recovery_s",
    "n_phone_warnings",
    "n_looking_away_warnings",
]
SECONDARY_OUTCOMES = ["n_long_warnings"]

OUTCOME_LABELS = {
    "pct_focused":             "Focused time (%)",
    "pct_warning":             "Warning time (%)",
    "total_warning_s":         "Total off-task time (s)",
    "n_warnings":              "Total warnings (n)",
    "mean_recovery_s":         "Mean warning duration (s)",
    "n_phone_warnings":        "Phone warnings (n)",
    "n_looking_away_warnings": "Looking-away warnings (n)",
    "n_long_warnings":         "Long warnings >= 10 s (n)",
}


def analyse_session(session_dir: Path) -> dict | None:
    try:
        meta = json.loads((session_dir / "meta.json").read_text())
        df = pd.read_csv(session_dir / "frames.csv")
        events = [json.loads(l) for l in open(session_dir / "events.jsonl")]
    except Exception:
        print(f"  skip: {session_dir.name}")
        return None

    pid = meta.get("participant_id", session_dir.name)
    cond = meta.get("condition", "unknown")

    task_start = next((e for e in events if e["event"] == "task_start"), None)
    task_end   = next((e for e in events if e["event"] == "task_end"),   None)

    if task_start and task_end and (task_end["t_mono"] - task_start["t_mono"]) >= 60.0:
        t0 = task_start["t_mono"]
        task_markers_ok = True
    else:
        t0 = max(10.0, df["t_mono"].min())
        task_markers_ok = False

    t1 = t0 + T_WINDOW_S
    actual_end = df["t_mono"].max()
    window_truncated = t1 > actual_end
    if window_truncated:
        t1 = actual_end
        print(f"  short: {pid}")

    task_df = df[(df["t_mono"] >= t0) & (df["t_mono"] <= t1)].copy()
    if len(task_df) == 0:
        print(f"  skip: {pid}")
        return None

    duration_s = t1 - t0
    n_frames = len(task_df)

    pct_focused = 100.0 * (task_df["main_state"] == "focused").sum() / n_frames
    pct_warning = 100.0 * (task_df["main_state"] == "warning").sum() / n_frames

    warn_events = [e for e in events
                   if e["event"] == "warning_triggered"
                   and t0 <= e.get("t_mono", -1) <= t1]
    resolve_events = [e for e in events
                      if e["event"] == "warning_resolved"
                      and t0 <= e.get("t_mono", -1) <= t1]

    n_warnings = len(warn_events)

    def _norm(reason):
        return (reason or "").lower().replace("_", " ").strip()

    n_phone_warnings = sum(1 for e in warn_events
                           if "phone" in _norm(e.get("reason")))
    n_looking_away_warnings = sum(1 for e in warn_events
                                  if "looking away" in _norm(e.get("reason")))

    durations = [e.get("duration") for e in resolve_events
                 if e.get("duration") is not None]
    if durations:
        total_warning_s = float(np.sum(durations))
        mean_recovery_s = float(np.mean(durations))
    else:
        total_warning_s = (task_df["main_state"] == "warning").sum() / 11.0
        mean_recovery_s = float("nan")

    n_long_warnings = sum(1 for d in durations if d >= LONG_WARN_THRESHOLD_S)

    robot_dispatch = [e for e in events
                      if e["event"] == "robot_dispatch"
                      and e.get("action") == "LOOK"
                      and t0 <= e.get("t_mono", -1) <= t1]

    return {
        "participant_id": pid,
        "condition": cond,
        "code_version": meta.get("code_version", ""),
        "task_markers_ok": task_markers_ok,
        "window_truncated": window_truncated,
        "duration_s": round(duration_s, 1),
        "n_frames": n_frames,
        "pct_focused": round(pct_focused, 2),
        "pct_warning": round(pct_warning, 2),
        "n_warnings": n_warnings,
        "n_phone_warnings": n_phone_warnings,
        "n_looking_away_warnings": n_looking_away_warnings,
        "n_long_warnings": n_long_warnings,
        "total_warning_s": round(total_warning_s, 2),
        "mean_recovery_s": (round(mean_recovery_s, 2)
                            if not np.isnan(mean_recovery_s) else None),
        "robot_interventions": len(robot_dispatch),
    }


def hedges_g(active, passive):
    a = np.asarray(active, dtype=float)
    b = np.asarray(passive, dtype=float)
    n1, n2 = len(a), len(b)
    if n1 < 2 or n2 < 2:
        return float("nan")
    s1, s2 = a.std(ddof=1), b.std(ddof=1)
    pooled_sd = np.sqrt(((n1 - 1) * s1**2 + (n2 - 1) * s2**2) / (n1 + n2 - 2))
    if pooled_sd == 0:
        return float("nan")
    d = (a.mean() - b.mean()) / pooled_sd
    J = 1.0 - 3.0 / (4.0 * (n1 + n2) - 9.0)
    return float(d * J)


def holm_bonferroni(pvals):
    p = np.asarray(pvals, dtype=float)
    n = len(p)
    order = np.argsort(p)
    adjusted = np.empty(n)
    running_max = 0.0
    for rank, idx in enumerate(order):
        adj = p[idx] * (n - rank)
        running_max = max(running_max, adj)
        adjusted[idx] = min(running_max, 1.0)
    return adjusted


def welch_with_normality_check(active, passive):
    a = np.asarray(active, dtype=float)
    b = np.asarray(passive, dtype=float)
    if len(a) < 3 or len(b) < 3:
        return None
    try:
        if shapiro(a).pvalue <= ALPHA or shapiro(b).pvalue <= ALPHA:
            return None
        t_res = ttest_ind(a, b, equal_var=False)
        return float(t_res.pvalue)
    except Exception:
        return None


def summarise_by_condition(df: pd.DataFrame) -> pd.DataFrame:
    rows = []
    measures = PRIMARY_OUTCOMES + SECONDARY_OUTCOMES
    for cond, grp in df.groupby("condition"):
        for m in measures:
            series = pd.to_numeric(grp[m], errors="coerce").dropna()
            if len(series) == 0:
                continue
            rows.append({
                "condition": cond,
                "measure":   m,
                "n":         len(series),
                "mean":      round(series.mean(), 3),
                "sd":        (round(series.std(ddof=1), 3) if len(series) > 1 else None),
                "median":    round(series.median(), 3),
                "min":       round(series.min(), 3),
                "max":       round(series.max(), 3),
            })
    return pd.DataFrame(rows)


def run_stats_tests(df: pd.DataFrame) -> list[str]:
    lines = []
    lines.append("=" * 95)
    lines.append("TABLE 5.2 — Group-level behavioural outcomes (matched 420-s window)")
    lines.append("=" * 95)
    lines.append("Mann-Whitney U (exact, two-tailed). pHolm = Holm-Bonferroni across the")
    lines.append("seven primary outcomes. pt = Welch's t-test reported only when both groups")
    lines.append("pass the Shapiro-Wilk normality assumption (p > .05). g = Hedges' g")
    lines.append("(small-sample corrected; positive = active higher).")
    lines.append("")

    on  = df[df["condition"] == "robot_on"]
    off = df[df["condition"] == "robot_off"]
    n1, n2 = len(on), len(off)
    lines.append(f"Sample sizes: robot_on n = {n1}, robot_off n = {n2}")
    lines.append("")

    if n1 < 2 or n2 < 2:
        lines.append("Insufficient sample size for between-group tests.")
        return lines

    raw = {}
    for m in PRIMARY_OUTCOMES + SECONDARY_OUTCOMES:
        a = pd.to_numeric(on[m],  errors="coerce").dropna().values
        b = pd.to_numeric(off[m], errors="coerce").dropna().values
        if len(a) < 2 or len(b) < 2:
            raw[m] = None
            continue
        try:
            mw = mannwhitneyu(a, b, alternative="two-sided", method="exact")
            U = float(mw.statistic)
            p_mw = float(mw.pvalue)
        except Exception:
            U, p_mw = float("nan"), float("nan")
        p_t = welch_with_normality_check(a, b)
        g = hedges_g(a, b)
        raw[m] = {
            "a_mean": a.mean(), "a_sd": a.std(ddof=1),
            "b_mean": b.mean(), "b_sd": b.std(ddof=1),
            "U": U, "p_mw": p_mw, "p_t": p_t, "g": g,
        }

    primary_p = [raw[m]["p_mw"] for m in PRIMARY_OUTCOMES if raw[m] is not None]
    primary_keys = [m for m in PRIMARY_OUTCOMES if raw[m] is not None]
    holm_adjusted = holm_bonferroni(primary_p)
    holm_lookup = dict(zip(primary_keys, holm_adjusted))

    header = (f"{'Outcome':<28} {'Active (M±SD)':>15} {'Passive (M±SD)':>17} "
              f"{'U':>5} {'pMW':>7} {'pHolm':>7} {'pt':>7} {'g':>7}")
    lines.append(header)
    lines.append("-" * len(header))

    def _fmt_p(p):
        if p is None or (isinstance(p, float) and np.isnan(p)):
            return "  —  "
        return f"{p:.3f}".lstrip("0") if p < 1 else f"{p:.3f}"

    def _row(measure, holm_p):
        r = raw.get(measure)
        if r is None:
            return f"{OUTCOME_LABELS[measure]:<28} (insufficient data)"
        a_str = f"{r['a_mean']:.2f} ± {r['a_sd']:.2f}"
        b_str = f"{r['b_mean']:.2f} ± {r['b_sd']:.2f}"
        return (f"{OUTCOME_LABELS[measure]:<28} {a_str:>15} {b_str:>17} "
                f"{r['U']:>5.0f} {_fmt_p(r['p_mw']):>7} "
                f"{_fmt_p(holm_p):>7} {_fmt_p(r['p_t']):>7} "
                f"{r['g']:>+7.2f}")

    for m in PRIMARY_OUTCOMES:
        lines.append(_row(m, holm_lookup.get(m)))

    lines.append("")
    lines.append("Planned secondary analysis (excluded from Holm family):")
    for m in SECONDARY_OUTCOMES:
        lines.append(_row(m, None))

    lines.append("")
    lines.append("Significance markers on raw pMW:")
    for m in PRIMARY_OUTCOMES + SECONDARY_OUTCOMES:
        r = raw.get(m)
        if r is None:
            continue
        p = r["p_mw"]
        sig = "***" if p < .001 else "**" if p < .01 else "*" if p < .05 \
              else "†" if p < .10 else ""
        if sig:
            lines.append(f"  {OUTCOME_LABELS[m]:<28}  pMW = {p:.4f}  {sig}")

    lines.append("")
    lines.append(f"Note: with N = {n1} per group, the smallest exact two-tailed p attainable")
    lines.append("from the Mann-Whitney U distribution is approximately .002 (perfect")
    lines.append("separation). Several outcomes will tie at p = .065 due to discreteness.")
    return lines


def plot_warning_rate_bars(df: pd.DataFrame, out: Path):
    fig, ax = plt.subplots(figsize=(6.5, 4))
    conds = sorted(df["condition"].unique())
    colors = {"robot_on": "#D97757", "robot_off": "#888888"}

    positions = np.arange(len(conds))
    for i, c in enumerate(conds):
        vals = pd.to_numeric(df[df["condition"] == c]["n_warnings"],
                              errors="coerce").dropna()
        jitter = np.random.uniform(-0.12, 0.12, len(vals))
        ax.scatter(positions[i] + jitter, vals,
                   color=colors.get(c, "#888888"), alpha=0.6, s=50, zorder=3)
        if len(vals):
            ax.hlines(vals.median(), positions[i] - 0.25, positions[i] + 0.25,
                      color=colors.get(c, "#444444"), lw=3, zorder=4)

    ax.set_xticks(positions)
    ax.set_xticklabels(conds)
    ax.set_ylabel("Number of warnings (420-s window)")
    ax.set_title("Warning count by condition\n(dots = individuals, bars = medians)")
    ax.grid(axis="y", alpha=0.3)
    plt.tight_layout()
    plt.savefig(out, dpi=150)
    plt.close()


def plot_distraction_profile(session_dirs: list[Path], out: Path):
    bin_size = 30.0
    n_bins = 14
    profiles = {"robot_on": [], "robot_off": []}

    for d in session_dirs:
        try:
            meta = json.loads((d / "meta.json").read_text())
            df = pd.read_csv(d / "frames.csv")
            events = [json.loads(l) for l in open(d / "events.jsonl")]
        except Exception:
            continue
        cond = meta.get("condition")
        if cond not in profiles:
            continue

        ts = next((e for e in events if e["event"] == "task_start"), None)
        te = next((e for e in events if e["event"] == "task_end"),   None)
        if ts and te and (te["t_mono"] - ts["t_mono"]) >= 60.0:
            t0 = ts["t_mono"]
        else:
            t0 = max(10.0, df["t_mono"].min())
        t1 = t0 + T_WINDOW_S

        task_df = df[(df["t_mono"] >= t0) & (df["t_mono"] <= t1)].copy()
        if len(task_df) == 0:
            continue
        task_df["bin"] = ((task_df["t_mono"] - t0) // bin_size).astype(int)
        profile = []
        for b in range(n_bins):
            bin_rows = task_df[task_df["bin"] == b]
            if len(bin_rows) == 0:
                profile.append(np.nan)
            else:
                profile.append(100.0 * (bin_rows["main_state"] == "warning").mean())
        profiles[cond].append(profile)

    fig, ax = plt.subplots(figsize=(8, 4.5))
    colors = {"robot_on": "#D97757", "robot_off": "#888888"}
    x = np.arange(n_bins) * bin_size / 60.0

    for cond, lst in profiles.items():
        if not lst:
            continue
        arr = np.array(lst, dtype=float)
        with np.errstate(all="ignore"):
            mean_prof = np.nanmean(arr, axis=0)
            sem = np.nanstd(arr, axis=0) / np.sqrt(np.sum(~np.isnan(arr), axis=0))
        ax.plot(x, mean_prof, color=colors[cond], lw=2, marker="o",
                label=f"{cond} (n={len(lst)})")
        ax.fill_between(x, mean_prof - sem, mean_prof + sem,
                        color=colors[cond], alpha=0.2)

    ax.set_xlabel("Time into task (minutes)")
    ax.set_ylabel("Time in warning state (%)")
    ax.set_title("Temporal distraction profile by condition\n(mean ± SEM per 30-s bin)")
    ax.legend()
    ax.grid(alpha=0.3)
    ax.set_xlim(0, 7)
    plt.tight_layout()
    plt.savefig(out, dpi=150)
    plt.close()


def plot_perceived_duration(perceived_df: pd.DataFrame | None, out: Path):
    fig, ax = plt.subplots(figsize=(6.5, 4))

    if perceived_df is None or len(perceived_df) == 0:
        ax.text(0.5, 0.5, "No survey data loaded\n"
                          "(run with --survey survey.csv to enable)",
                ha="center", va="center", transform=ax.transAxes,
                fontsize=12, color="#888")
        ax.set_axis_off()
        plt.tight_layout()
        plt.savefig(out, dpi=150)
        plt.close()
        return

    conds = sorted(perceived_df["condition"].unique())
    colors = {"robot_on": "#D97757", "robot_off": "#888888"}
    positions = np.arange(len(conds))

    for i, c in enumerate(conds):
        vals = pd.to_numeric(perceived_df[perceived_df["condition"] == c]["perceived_duration"],
                              errors="coerce").dropna()
        jitter = np.random.uniform(-0.12, 0.12, len(vals))
        ax.scatter(positions[i] + jitter, vals,
                   color=colors.get(c, "#888888"), alpha=0.6, s=50, zorder=3)
        if len(vals):
            ax.hlines(vals.median(), positions[i] - 0.25, positions[i] + 0.25,
                      color=colors.get(c, "#444444"), lw=3, zorder=4)

    ax.axhline(7.0, color="black", lw=1, ls="--", alpha=0.6)
    ax.text(len(conds) - 0.5, 7.1, "actual (7 min)", fontsize=9, color="#444")
    ax.set_xticks(positions)
    ax.set_xticklabels(conds)
    ax.set_ylabel("Perceived duration (minutes)")
    ax.set_title("Perceived task duration by condition\n(dashed = actual 7 min)")
    ax.grid(axis="y", alpha=0.3)
    plt.tight_layout()
    plt.savefig(out, dpi=150)
    plt.close()


def write_summary_report(df: pd.DataFrame, summary: pd.DataFrame,
                          stats_lines: list[str], out: Path):
    lines = []
    lines.append("# User Study — Analysis Summary\n")
    lines.append(f"Generated: {pd.Timestamp.now().strftime('%Y-%m-%d %H:%M')}\n")

    lines.append("## Participants\n")
    n_total = len(df)
    by_cond = df["condition"].value_counts().to_dict()
    lines.append(f"- **Total participants**: {n_total}")
    for c, n in by_cond.items():
        lines.append(f"- **{c}**: n = {n}")
    lines.append("")

    bad = df[~df["task_markers_ok"]]
    if len(bad):
        lines.append("## Data-quality notes\n")
        lines.append(f"- **{len(bad)} session(s)** missing task markers. "
                     f"Task window inferred from session bounds: "
                     f"{', '.join(bad['participant_id'].tolist())}")
        lines.append("")

    truncated = df[df["window_truncated"]]
    if len(truncated):
        lines.append(f"- **{len(truncated)} session(s)** shorter than {T_WINDOW_S:.0f}s "
                     f"matched window: "
                     f"{', '.join(truncated['participant_id'].tolist())}")
        lines.append("")

    versions = df["code_version"].value_counts().to_dict()
    if len(versions) > 1:
        lines.append("- **Multiple code versions** detected: " +
                     ", ".join(f"{v} ({n})" for v, n in versions.items()))
        lines.append("")

    lines.append("## Key measures (median by condition)\n")
    pivot = summary.pivot(index="measure", columns="condition", values="median")
    lines.append(pivot.to_markdown())
    lines.append("")

    lines.append("## Group-level statistical tests\n")
    lines.append("```")
    lines.extend(stats_lines)
    lines.append("```\n")

    lines.append("## Figures\n")
    lines.append("- `fig_warning_rates.png` — warning count by condition")
    lines.append("- `fig_distraction_profile.png` — % time in warning state per 30-s bin")
    lines.append("- `fig_perceived_duration.png` — subjective duration by condition")
    lines.append("")

    out.write_text("\n".join(lines))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--logs", default="logs")
    ap.add_argument("--out",  default="analysis")
    ap.add_argument("--exclude", nargs="*", default=[])
    ap.add_argument("--survey", default=None)
    args = ap.parse_args()

    logs_dir = Path(args.logs)
    out_dir  = Path(args.out)
    out_dir.mkdir(parents=True, exist_ok=True)

    if not logs_dir.exists():
        print(f"log folder not found: {logs_dir}")
        return

    session_dirs = [d for d in logs_dir.iterdir()
                    if d.is_dir() and (d / "meta.json").exists()]
    session_dirs.sort()
    print(f"loaded {len(session_dirs)} sessions")

    rows = []
    for d in session_dirs:
        row = analyse_session(d)
        if row is None:
            continue
        if row["participant_id"] in args.exclude:
            continue
        rows.append(row)

    if not rows:
        print("no valid sessions")
        return

    df = pd.DataFrame(rows)
    df.to_csv(out_dir / "per_participant.csv", index=False)
    summary = summarise_by_condition(df)
    summary.to_csv(out_dir / "condition_summary.csv", index=False)
    stats_lines = run_stats_tests(df)
    (out_dir / "stats_tests.txt").write_text("\n".join(stats_lines))

    perceived_df = None
    if args.survey and Path(args.survey).exists():
        perceived_df = pd.read_csv(args.survey)
        perceived_df = perceived_df.merge(df[["participant_id", "condition"]],
                                          on="participant_id", how="left")

    plot_warning_rate_bars(df, out_dir / "fig_warning_rates.png")
    plot_distraction_profile(session_dirs, out_dir / "fig_distraction_profile.png")
    plot_perceived_duration(perceived_df, out_dir / "fig_perceived_duration.png")
    write_summary_report(df, summary, stats_lines, out_dir / "summary_report.md")

    print(f"done. output in {out_dir}/")


if __name__ == "__main__":
    main()
