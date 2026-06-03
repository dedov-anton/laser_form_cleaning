# Доработки — журнал изменений

Хронология задач, подход к решению и результат.

---

## 2026-06-02 — Тестовая УП: дуга кольца movec (фаза 1)

### Задача

Очистка верх/низ кольца по **дуге** (`movec`), а не только радиальными `movel`. Из‑за 6 осей полный круг — двумя полуокружностями (фаза 2). Сейчас — **одна** тестовая дуга CW **180° → 90° → 0°** по средней линии кольца для проверки на роботе.

### Решение

- [`src/exporters/robot/ring_arc.py`](src/exporters/robot/ring_arc.py) — центр из `frame_pose_applied`, радиус `inner + ring_width/2`, касательная CW, заготовка `arc_track_radii_mm` для нескольких дорожек.
- [`build_ring_movec_test_program`](src/exporters/robot/elite_program.py) — `movel` на 180°, `movec(p_via, p_to, a, v, r, mode)` via 90° → 0° (`mode=0` фикс., без параметра `t` по CS Script Manual §2.1.2), возврат на 180°.
- [`scripts/build_ring_movec_test_up.py`](scripts/build_ring_movec_test_up.py) — читает [`config/trajectories/bottom_ring.json`](config/trajectories/bottom_ring.json).
- Выход: [`УП/test_ring_movec_180_to_0.txt`](УП/test_ring_movec_180_to_0.txt).

### Запуск

```bash
python scripts/build_ring_movec_test_up.py
```

### Фаза 2 (2026-06-02) — GUI и полная УП дугами

- [x] Секция «Низ формы»: строка **«Дуги: …»** (1 проход или N + перекрытие мм/%), цвет как у секторов.
- [x] Кнопка **«Создать УП дугами»** → [`export_ring_arc_program`](src/exporters/robot/export.py).
- [x] [`plan_arc_passes`](src/exporters/robot/ring_arc.py) + [`build_ring_arc_program`](src/exporters/robot/elite_program.py): на каждый радиус CW (180→90→0) и CCW (180→270→0).
- [ ] `top_ring` — после появления генератора.
- [ ] STEP для дуг.

### Поза из GUI, старт/финиш без переноса (2026-06-02)

- **Дуговая УП** не требует «Создать траекторию» / «Переместить»: геометрия из **6D позы** в шапке GUI + параметры кольца; старт/финиш из полей секции (мм, как в `project.json`).
- [`build_ring_arc_geometry`](src/exporters/robot/ring_arc.py) — дуга в локальной СК кольца, перенос `transform_point` / `transform_vector` с полным Rx Ry Rz.
- [`transform_work_trajectory`](src/transforms/trajectory_transform.py) — старт/финиш и соответствующие концы `approach`/`departure` **не** переносятся; рабочие точки и дуги — переносятся.
- Радиальная «Создать УП» подставляет старт/финиш из GUI в момент экспорта ([`apply_program_start_finish`](src/transforms/trajectory_transform.py)).

---

## 2026-06-02 — Коррекция запястья cylinder_wall, Lcorr, наклоны, RPY Elite

### Задачи

1. **Ориентация УП Elite** — на реальном роботе `rx=ry=rz=0` соответствует инструменту строго вниз; экспорт RPY из 6D должен совпадать с калибровкой (`УП/test_vertical_down.txt`), а не с ошибочной схемой rotvec.
2. **Касательная пути** — `tool_axis_x` на цилиндре должен следовать направлению вертикального прохода (top → profile → bottom), а не быть константой.
3. **Экспериментальная коррекция запястья** — лазер на голове отведён на 90°: отдельный pipeline (не в основном «Создать УП») для перевода ориентации «радиаль → вертикаль вверх» + JSON / STEP / УП.
4. **Смещение TCP (Lcorr)** — от точки на стенке: `Lcorr` вдоль наклонённого луча (`−tool_axis_z`) к оси, затем `Lcorr` по направлению «вверх», перпендикулярному лучу в плоскости луч–Z (`tcp_position_after_lcorr` в `wrist_correction.py`). Точки на стенке в `work_segments` не сдвигаются.
5. **Наклоны и промежуточные точки** — после коррекции запястья сохранить tilt из генератора (в плоскости радиаль–Z он становится «вперёд/назад» относительно вертикали) и все стопы `work_profile_*`.

### Проблемы и как решали

| Проблема | Решение |
|----------|---------|
| RPY на роботе «ломались» (например `ry≈3.14` вместо ~0) | `rotation_matrix_to_rpy` по соглашению bk (`column_stack`); `tcp_z = -tool_axis_z` под калибровку vertical-down |
| Глобальный поворот RPY `(1.57,0,-1.57)→(3.14,0,1.57)` на всю траекторию давал бессмыслицу в STEP | Отказ от глобальной RPY-коррекции; геометрический поворот **per pose** в `wrist_correction.py` |
| Коррекция сбрасывала `top_tilt` / profile tilt | Поворот всего кадра: **outward → (0,0,+1)** применяется к `tool_axis_z` и `tool_axis_x`, а не `tool_axis_z = (0,0,1)` |
| Ось поворота была `inward` вместо луча наружу | `outward_direction_from_axis` = от оси к стенке; Rodrigues от outward к вертикали |
| TCP не совпадает с точкой луча на стенке | **Lcorr (мм)** в GUI: двухшаговое смещение по `tool_axis_z` (с наклоном) и перпендикуляру в плоскости луч–Z; не горизонтальный inward + Z |
| STEP после коррекции запястья не совпадал с УП | STEP только из УП: точки `pose_work_*` + одна стрелка TCP+Z из `rx,ry,rz`, без линий между точками (`from_elite_program.py`) |
| Траектории раздували `project.json` | Файлы в `config/trajectories/{id}.json`, в проекте — пути `trajectory_files` |

### Решение (реализация)

- `src/exporters/robot/orientation.py` — исправленный RPY + mount rotation из GUI.
- `src/generators/cylinder_wall/calc.py` — `tool_axis_x` из `stop_path_tangent`.
- `src/transforms/wrist_correction.py` — radial→vertical upward, Lcorr, сохранение tilt через поворот кадра.
- `src/transforms/wrist_correction_export.py` — JSON + STEP + УП; `scripts/apply_wrist_correction.py`.
- GUI: кнопка «Коррекция запястья», поле Lcorr; выход: `cylinder_wall_wrist_corrected.*`.
- `src/storage/project_store.py` — внешние JSON траекторий, миграция embedded при save.
- Тесты: `tests/transforms/test_wrist_correction.py`, обновлены robot/cylinder tests.

### STEP после коррекции — только по УП (2026-06-02)

Эталон для проверки траектории — **УП Elite**, не оси `tool_axis_x/z` из JSON.

**Pipeline** ([`wrist_correction_export.py`](src/transforms/wrist_correction_export.py)):

1. `build_elite_program(corrected, robot_settings)` — тот же код, что «Создать УП».
2. Запись [`УП/cylinder_wall_wrist_corrected.txt`](УП/cylinder_wall_wrist_corrected.txt).
3. [`export_step_from_elite_program(program)`](src/exporters/step/from_elite_program.py) — парсинг `pose_work_N = [x,y,z,rx,ry,rz]` (м, рад).

**В STEP рисуется только:**

- маркер TCP (координаты из строки УП, мм);
- одна стрелка — **TCP +Z** из `rx,ry,rz` ([`rpy_to_rotation_rows`](src/exporters/robot/orientation.py)).

**Не рисуется:** линии между точками, travel/work/reference, оси из JSON.

**Обычный STEP** (без `wrist_correction_applied`) — без изменений ([`export_trajectory`](src/exporters/step/export.py)). Прямой экспорт corrected JSON в старый STEP запрещён (ошибка с подсказкой).

**Тесты:** [`tests/exporters/test_step_wrist_corrected.py`](tests/exporters/test_step_wrist_corrected.py).

### Итог

- [x] УП cylinder_wall: осмысленные RPY при вертикальной калибровке TCP.
- [x] Коррекция запястья (эксперимент): ориентация вверх, Lcorr, tilt и profile-стопы сохраняются.
- [x] STEP после коррекции совпадает с УП (точка + вектор TCP+Z, без переходов).
- [x] Полный pytest: **104+ passed** (1 старый fail в `test_geometry_from_bottom_ring_json`, z фикстуры).
- [ ] Сверка на реальном роботе с разными Lcorr и наклонами — по результатам прогона пользователя.

### Отложено

- Включение коррекции запястья в основной «Создать УП» (пока только отдельная кнопка / скрипт).
- `top_ring` — коррекция запястья не делалась.

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
- [x] Генератор **поверхности качения** (`cylinder_wall`) — см. запись ниже.
- [x] Экспорт **УП робота Elite** — см. запись ниже (2026-05-30).
- [x] Кнопка «Переместить» в секции cylinder (top — при реализации генератора).
- [ ] Сверка `rotation_matrix_sxyz` с реальной TCP-позой из Elite (пример от пользователя).

---

## 2026-05-30 — Генератор поверхности качения (cylinder_wall)

### Задачи

1. **Вертикальные проходы** по внутренней стенке цилиндра — сверху вниз, обработка изнутри.
2. **8 секторов по 45°** — каждый сектор обрабатывается отдельно; стартовый сектор и направление (CW/CCW).
3. **Авто-расчёт проходов** в секторе по ширине луча с ручной коррекцией и индикатором перекрытия.
4. **Наклон инструмента** — отдельно для верхней и нижней точки прохода; ось Z к центру цилиндра.
5. **Старт/финиш** — отдельные поля в каждой секции GUI (не в общих параметрах).

### Как решалось

- Модуль `src/generators/cylinder_wall/` — params, calc, models, adapter, generator.
- Общие хелперы ориентации: `src/common/tool_orientation.py` (`tilt_tool_axis_inward`, `orthogonal_tool_axis_x`).
- Геометрия: `z_top` — расстояние до верхней кромки; низ = `z_top − height`; точки на `inner_radius`.
- Авто-проходы: `floor(дуга_45° / beam_width) + 1`, перекрытие = `beam − дуга/n`.
- Порядок: секторы с `start_sector` по/против часовой; внутри сектора — равномерные углы; между проходами — прямой transfer.
- Reference geometry: окружность + 8 радиальных границ секторов (polylines).
- GUI: полная секция с кнопками create / move / STEP / УП.
- Миграция: старт/финиш из top-level `project.json` → `bottom_ring` при загрузке.

### Что сделано (итог)

- [x] `cylinder_wall` — расчёт, GUI, STEP, перенос по 6D.
- [x] Старт/финиш перенесены в секции bottom_ring и cylinder_wall.
- [x] Тесты `tests/generators/cylinder_wall/test_calc.py`.

---

## 2026-05-30 — Профиль боковой стенки (cylinder_wall)

### Задача

Два промежуточных waypoint на вертикальном проходе (как профиль кольца): смещение **вниз от верха**, **к оси от стенки**, наклон; превью-сечение в GUI.

### Проблемы и как решали

| Проблема | Решение |
|----------|---------|
| Вертикальный проход был только «верх → низ», без промежуточных точек | `WallProfileWaypoint` + `build_pass_stops()`: верх → P1/P2 (по `z_down_mm`) → низ |
| Нужно задавать смещение **от стенки к оси** и **наружу** (как у кольца) | `radial_inward_mm > 0` — к оси, `< 0` — от стенки; радиус = `inner_radius − radial_inward` |
| Превью профиля не обновлялось при вводе | `_bind_updates()` в `app.py` + `_update_profile_preview()` в секции cylinder_wall |
| Стрелки наклона на превью смотрели «от стенки внутрь», а не наоборот | Зеркалирование направления стрелок только в `WallProfileCanvas` (расчёт и STEP не трогали) |
| Падение при старте GUI cylinder_wall | Добавлен недостающий импорт `parse_optional_float` |

### Решение (реализация)

- `WallProfileWaypoint`: `z_down_mm`, `radial_inward_mm`, `tilt_deg`; точка выкл. при `z_down_mm <= 0`.
- `build_pass_stops`: верх → профильные (по z_down) → низ; валидация z и радиуса.
- `WallProfileCanvas` — ось X к оси, Y вниз от верха, стрелки наклона через `tilt_tool_axis_radial`.
- GUI: блок «Профиль / наклон» с P1/P2 и интерактивным превью.

### Итог

- [x] Расчёт, GUI, save/load, тесты профиля (`tests/generators/cylinder_wall/test_calc.py`).

---

## 2026-05-30 — Экспорт УП Elite (robot export)

### Задача

Сгенерировать управляющую программу для робота Elite CS612 из `WorkTrajectory`: 6D-позы → `movel`, координаты в **метрах**, углы в **радианах**. Образец — FreeCAD-скрипты сварки.

### Проблемы и как решали

| Проблема | Решение |
|----------|---------|
| `welding_path_generator(1).py` тянет скомпилированный `elite_core`, его нет в репозитории | Взяли **чистый Python** из [`welding_path_generator_bk.py`](welding_path_generator_bk.py): `calculate_orientation`, формат `def welding_program(): … end` |
| Заглушка `export_robot_program` — кнопка «Создать УП» ничего не делала | Модули `src/exporters/robot/`: orientation → elite_program → export |
| Непонятно, из какой СК экспортировать | Только **мировая** траектория после «Переместить»; иначе ошибка с подсказкой |
| В bk есть смещение TCP и безопасная высота — перенесли в первой версии | **Убрано по требованию:** в этом проекте точки траектории уже в нужных координатах; подвод/отвод — явные start/finish |
| Параметры v, a, blend — где задавать | Блок «Параметры УП (Elite)» в шапке GUI + сохранение в `FormProject` / `project.json` |

### Решение (реализация)

- `settings.py` — `RobotExportSettings`: blend (м), v (м/с), a (м/с²).
- `orientation.py` — `calculate_orientation` (Static XYZ, коррекция roll −90° из bk).
- `elite_program.py` — последовательность: **start → рабочие точки (с blend на промежуточных) → finish**.
- `export.py` — запись `.txt`, метаданные `# TRAJECTORY_LAYER: {generator_id}`.
- Кнопки «Создать УП» в `bottom_ring` и `cylinder_wall`.

### Итог

- [x] Экспорт УП из `WorkTrajectory` (проверено на bottom_ring — см. пример вывода).
- [x] Параметры movel в GUI, persist в проекте.
- [x] Тесты `tests/exporters/test_robot_export.py` — **51 passed** в полном прогоне.

### Отложено

- Плавный выход (`apply_finish_adjustments` из bk), `movej`, экспорт для `top_ring`.
- Сверка углов с реальной TCP-позой робота Elite.
