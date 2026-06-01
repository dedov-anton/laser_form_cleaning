# Доработки — журнал изменений

Хронология задач, подход к решению и результат.

---

## 2026-06-01 — Модульная архитектура + перенос траектории по 6D-позе

### Задачи

1. **Рефакторинг монолита** — код был «склеен» вокруг нижнего кольца (`src/core/`, `src/export/`, монолитный `app.py`). Нужна модульность под три поверхности (низ, верх, цилиндр) и общие экспортёры.
2. **Единое GUI** — одно окно, три секции, у каждой свой комплект кнопок (создать / STEP / УП).
3. **Общий формат результата** — `WorkTrajectory` как «сырой» набор векторов для STEP и будущей УП робота.
4. **Перенос в СК оснастки** — траектория строится в нуле; нужен модуль жёсткого переноса по 6D-позе (XYZ + Rx Ry Rz) в координаты матрицы/робота Elite.
5. **Сохранение проекта** — единый `config/project.json`, миграция из `last_params.json`.

### Как решалось

#### Модульная архитектура

| Было | Стало |
|------|--------|
| `src/core/models.py` + `ring_calc.py` | `src/generators/bottom_ring/` (params, calc, adapter, generator) |
| `src/export/` | `src/exporters/step/`, `src/exporters/robot/` (заглушка УП) |
| `src/core/config_store.py` | `src/storage/project_store.py` |
| Монолитный `app.py` (~600 строк) | `app.py` (shell) + `gui/sections/` + `gui/widgets/` |

- Общие типы: `src/common/` — `geometry.py`, `trajectory.py` (`WorkTrajectory`), `project.py` (`FormProject`), `generator.py` (Protocol).
- Генератор `bottom_ring` возвращает `WorkTrajectory` через адаптер `to_work_trajectory()`; математика проходов **не менялась**.
- Заглушки: `top_ring`, `cylinder_wall` — секции GUI с disabled-кнопками.
- Реестр: `src/generators/registry.py`.

#### Перенос по 6D-позе

- **Конвенция углов:** Static XYZ (Elite sxyz) — `R = Rz · Ry · Rx`, градусы; ориентир — `welding_path_generator(1).py` (roll/pitch/yaw через `elite_core`, не rotvec).
- **Модуль:** `src/common/frame_pose.py` (математика), `src/transforms/trajectory_transform.py` (`transform_work_trajectory`).
- **Поведение:**
  - «Создать траекторию» → локальная копия в нуле (`trajectories_local` + `trajectories`).
  - «Переместить по 6D позе» → новый `WorkTrajectory` из локальной копии; **локальная не затирается**; мировая перезаписывается; повторное нажатие — снова от локальной (без накопления).
- **GUI:** поля X Y Z Rx Ry Rz в «Общих параметрах»; кнопка «Переместить» в секции «Низ формы» (для top/cylinder — подключить при реализации генераторов, ядро уже универсальное).
- **STEP:** после переноса окружности `reference_geometry.circles` → полилинии; экспорт рисует и `polylines`.

### Что сделано (итог)

- [x] Модульная структура каталогов `common/`, `generators/`, `storage/`, `exporters/`, `transforms/`, `gui/sections/`.
- [x] `WorkTrajectory` — единый результат всех генераторов.
- [x] GUI: общие параметры, секции bottom / top / cylinder, три кнопки на секцию (УП — заглушка).
- [x] `config/project.json`, миграция `last_params.json`, слоты `trajectories` и `trajectories_local`.
- [x] Универсальный STEP-экспорт по `WorkTrajectory`.
- [x] Модуль переноса `transform_work_trajectory` + GUI для bottom_ring.
- [x] Тесты: 27 passed (`tests/generators/`, `tests/common/`, `tests/transforms/`).
- [x] README обновлён.

### Отложено / следующие шаги

- [ ] Генератор **верхнего кольца** (`top_ring`).
- [ ] Генератор **поверхности качения** (`cylinder_wall`).
- [ ] Экспорт **УП робота Elite** (по образцу `welding_path_generator(1).py` + `elite_core`).
- [ ] Кнопка «Переместить» в секциях top/cylinder (общий хелпер из bottom_ring).
- [ ] Сверка `rotation_matrix_sxyz` с реальной TCP-позой из Elite (пример от пользователя).
- [ ] Расширить `_transform_reference_geometry` под произвольные `polylines` новых генераторов.
