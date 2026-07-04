# Analysis

Reproduces the group-level behavioural results from the dissertation (Tables 5.1 and 5.2):
per-participant outcomes, Mann–Whitney U tests, Hedges' *g* effect sizes, and
Holm–Bonferroni correction across the seven primary outcomes.

## Running

```bash
pip install -r ../requirements.txt
python sessions_analysis.py --logs <path-to-session-logs>
```

> `sessions_analysis.py` regenerates the outputs below from the raw session logs. The raw logs are
> **not** included in this repository (human-subjects data); only the de-identified,
> aggregated outputs in [`data/`](data/) are shared.

## Contents of `data/` (de-identified, aggregated)

| File | Description |
|------|-------------|
| `per_participant.csv` | One row per participant (pseudonymous codes P06–P17), summary metrics only |
| `condition_summary.csv` | Group-level means/SD/median by condition (robot_on vs robot_off) |
| `summary_report.md` | Human-readable summary of participants, key measures, and tests |
| `stats_tests.txt` | Full Table 5.2 statistical output |
| `figures/` | Warning rates, distraction profile, perceived-duration plots |

## Key findings

Active (robot_on) vs passive (robot_off), matched 420 s window, N = 6 per group:

- Focused time: **75.98% → 91.88%** (Hedges' *g* = +1.60)
- Off-task time: **86.0 s → 21.6 s** (*g* = −1.68)
- Phone warnings: **8.33 → 2.17** (Mann–Whitney U p = .004, p_Holm = .030)

See [`data/summary_report.md`](data/summary_report.md) for the full tables.
