# HousdorfPolygonGen

Генератор пар полигонов **P0** (выпуклая оболочка) и **P** (невыпуклый контур с карманами) для бенчмарка `grid_plus_shor` и исследования УЛЛ.

## Пайплайн

1. Случайные точки в квадрате → convex hull → **P0**
2. `dent_count` карманов **вставкой на ребро** P0 (цепочка внутрь, случайное направление в допустимом конусе) → **P**
3. Проверки: `isSimple(P)`, `conv(P) ≈ P0`
4. Метрики `*_actual` и запись `metadata.csv` (колонки `*_err = |target - actual|`)
5. Файлы: `<case_id>_polygon_convex.txt` (P0), `<case_id>_polygon_nonconvex.txt` (P)
6. Опционально `--enrich-hausdorff`: в `metadata.csv` дописываются `grid_*` (grid+Shor) и `ull_*` (УЛЛ) — оптимальное расстояние Хаусдорфа и сдвиг `(shift_x, shift_y)`

Инвариант: \(\mathrm{conv}(P)=P_0\); convex-файл не пересчитывается после построения P.

### Метрики (actual)

| Ключ | Формула |
|------|---------|
| `depth_rel` | \(\max_{q\in P} \mathrm{dist}(q,\partial P_0) / D\), \(D=\sqrt{\mathrm{area}(P_0)}\) |
| `bridge_width_rel` | хорда кармана на \(P_0\) / \(\mathrm{perimeter}(P_0)\) |
| `pocket_width_rel` | ширина устья кармана / \(D\) |
| `area_ratio` | \(\mathrm{area}(P_0)/\mathrm{area}(P)\) |
| `alpha_proxy_actual` | max по reflex-вершинам \((\pi - \angle ABC)\); для sweep / `target_bins` |
| `alpha_lebedev_full` | полная α(M) Лебедева (2007): супремум углового конуса проекций; после генерации |

## Сборка

```bash
cmake -S . -B build
cmake --build build --config Debug
```

Исполняемый файл: `build/Debug/polygon_gen.exe` (MSVC) или `build/polygon_gen`.

## Запуск

```bash
polygon_gen --experiment experiment.json
polygon_gen --experiment experiment_ull_demo.json
polygon_gen --count 200 --threads 8 --seed 42
polygon_gen --experiment experiment.json --preview-every 50
```

`count: 0` в JSON — число кейсов = сумма уровней всех осей × `replicate` (см. `experiment_ull_demo.json`).

Параметры CLI: `--count`, `--threads`, `--out-dir`, `--seed`, `--square xmin xmax ymin ymax`, `--preview-every K`, `--enrich-hausdorff`, `--hausdorff-raster`, `--hausdorff-grid`.

Обогащение Хаусдорфа (нужны `scipy`, `numpy` и репозитории `grid_plus_shor`, `multi_step_method`):

```bash
polygon_gen --experiment experiment.json --enrich-hausdorff
# или офлайн:
python tools/enrich_metadata_hausdorff.py --run-dir run_YYYYMMDD_HHMMSS
```

Колонки в `metadata.csv`: `grid_hausdorff`, `grid_shift_x`, `grid_shift_y`, `ull_hausdorff`, `ull_shift_x`, `ull_shift_y`, `ull_vs_grid_rel_err`.

## Выход

```
run_YYYYMMDD_HHMMSS/
  experiment.json
  metadata.csv
  <case_id>/
    <case_id>_polygon_convex.txt
    <case_id>_polygon_nonconvex.txt
```

Разделитель CSV: `;`. Колонки target/actual для калибровки рычагов (см. `GENERATOR_IMPLEMENTATION_PLAN.md`).

## Бенчмарк Python

```bash
cd ../grid_plus_shor/src
python benchmark_polygon_files.py --root ../../HousdorfPolygonGen/run_YYYYMMDD_HHMMSS
```

## Режимы sweep (`experiment.json`)

| `sweep_mode`      | Описание                                      |
|-------------------|-----------------------------------------------|
| `one_at_a_time`   | Одна ось меняется, остальные — медиана        |
| `full`            | Декартово произведение уровней                |
| `lhs`             | Latin hypercube                               |
| `target_bins`     | До 3 попыток под `alpha_lebedev`              |

## График «метрика → Δ_УЛЛ»

1. Сгенерировать run: `polygon_gen --experiment experiment_ull_demo.json`
2. Прогнать `benchmark_polygon_files.py`, получить CSV с `case_id`, `f_ref`, `f_method` (или `rel_err`).
3. Построить scatter:

```bash
pip install matplotlib
python tools/plot_ull_sweep.py --run-dir run_YYYYMMDD_HHMMSS \
  --benchmark-csv path/to/benchmark.csv --metric depth_rel_actual
```

Джойн по `case_id`; по оси X — `*_actual` из `metadata.csv`, по Y — \(|f_{ULL}-f_{ref}|/f_{ref}\).
