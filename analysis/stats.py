"""
stats.py — batch analysis of user-study session logs.

Reproduces Tables 5.1 and 5.2 from the dissertation:
  - per-participant behavioural outcomes
  - group-level Mann-Whitney U tests
  - Hedges' g effect sizes (small-sample corrected)
  - Holm-Bonferroni correction across the seven primary outcomes

Usage:
    python stats.py --logs <path-to-session-logs>

Outputs the de-identified aggregates found in analysis/data/.

------------------------------------------------------------------------------
TODO: replace this placeholder with the actual analysis script
(sessions_analysis.py / analyze_sessions.py). Raw session logs are NOT part of
this repository; only the aggregated outputs in analysis/data/ are shared.
------------------------------------------------------------------------------
"""

if __name__ == "__main__":
    raise SystemExit(
        "Placeholder — add the real analysis script here "
        "(see analysis/data/ for the generated outputs)."
    )
