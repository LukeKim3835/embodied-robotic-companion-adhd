# User Study — Analysis Summary

Generated: 2026-05-05 11:57

## Participants

- **Total participants**: 12
- **robot_on**: n = 6
- **robot_off**: n = 6

## Data-quality notes

- **1 session(s)** missing task markers. Task window inferred from session bounds: P06

## Key measures (median by condition)

| measure                 |   robot_off |   robot_on |
|:------------------------|------------:|-----------:|
| mean_recovery_s         |       7.745 |      3.325 |
| n_long_warnings         |       4.5   |      0     |
| n_looking_away_warnings |       3.5   |      3.5   |
| n_phone_warnings        |       6     |      2     |
| n_warnings              |       9     |      6     |
| pct_focused             |      76.665 |     92.345 |
| pct_warning             |      19.82  |      4.975 |
| total_warning_s         |      82.775 |     21.085 |

## Group-level statistical tests

```
===============================================================================================
TABLE 5.2 — Group-level behavioural outcomes (matched 420-s window)
===============================================================================================
Mann-Whitney U (exact, two-tailed). pHolm = Holm-Bonferroni across the
seven primary outcomes. pt = Welch's t-test reported only when both groups
pass the Shapiro-Wilk normality assumption (p > .05). g = Hedges' g
(small-sample corrected; positive = active higher).

Sample sizes: robot_on n = 6, robot_off n = 6

Outcome                        Active (M±SD)    Passive (M±SD)     U     pMW   pHolm      pt       g
----------------------------------------------------------------------------------------------------
Focused time (%)                91.88 ± 2.78     75.98 ± 12.69    30    .065    .390    .027   +1.60
Warning time (%)                 5.14 ± 2.43     20.54 ± 11.70     6    .065    .390    .022   -1.68
Total off-task time (s)        21.62 ± 10.10     86.02 ± 48.93     6    .065    .390    .022   -1.68
Total warnings (n)               6.00 ± 2.53      12.33 ± 7.06     6    .065    .390    .082   -1.10
Mean warning duration (s)        3.65 ± 1.20       6.95 ± 3.22     6    .065    .390    .054   -1.26
Phone warnings (n)               2.17 ± 0.75       8.33 ± 6.15     1    .004    .030    .057   -1.30
Looking-away warnings (n)        3.67 ± 2.16       4.00 ± 2.61    17    .937    .937    .814   -0.13

Planned secondary analysis (excluded from Holm family):
Long warnings >= 10 s (n)        0.33 ± 0.52       3.50 ± 2.07     4    .026     —       —     -1.93

Significance markers on raw pMW:
  Focused time (%)              pMW = 0.0649  †
  Warning time (%)              pMW = 0.0649  †
  Total off-task time (s)       pMW = 0.0649  †
  Total warnings (n)            pMW = 0.0649  †
  Mean warning duration (s)     pMW = 0.0649  †
  Phone warnings (n)            pMW = 0.0043  **
  Long warnings >= 10 s (n)     pMW = 0.0260  *

Note: with N = 6 per group, the smallest exact two-tailed p attainable
from the Mann-Whitney U distribution is approximately .002 (perfect
separation). Several outcomes will tie at p = .065 due to discreteness.
```

## Figures

- `fig_warning_rates.png` — warning count by condition
- `fig_distraction_profile.png` — % time in warning state per 30-s bin
- `fig_perceived_duration.png` — subjective duration by condition
