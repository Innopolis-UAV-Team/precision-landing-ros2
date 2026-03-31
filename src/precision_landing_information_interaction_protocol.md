Протокол информационного взаимодействия

Версия протокола: 1.0
Дата: 31.03.2026

## 1. Назначение

Документ фиксирует информационное взаимодействие между узлами ROS 2 в проекте `precision-landing-ros2` для сценария точной посадки БВС по ArUco-меткам.

## 2. Состав подсистемы

В рабочем контуре участвуют два основных пакета:

- `aruco_tracker` (детекция метки/площадки и публикация цели посадки)
- `precision_landing_ros2` (контур управления посадкой через MAVROS)

Дополнительно используется `vision_opencv` как внешний стек компьютерного зрения.

## 3. Узлы и роли

### 3.1 `landing_detector` (`aruco_tracker/landing_detector_node`)

Роль:

- принимает изображение из топиков `image_raw/compressed` и `camera_info`
- детектирует ArUco-маркеры по конфигурации доски
- публикует целевую позпозицию для MAVROS в `/mavros/landing_target/pose`
- публикует TF маркеров и сглаженный TF посадочной площадки
- публикует отладочное изображение `debug_image/compressed`

### 3.2 `precision_lander_node` (`precision_landing_ros2/precision_lander_node`)

Роль:

- получает телеметрию MAVROS и целевую позицию площадки
- следит за состоянием точной посадки
- генерирует скорости в OFFBOARD через `/mavros/setpoint_velocity/cmd_vel`
- вызывает сервисы MAVROS для переключения режима и дизарма

## 4. Топики (контур обмена)

### 4.1 Входящие топики `landing_detector`

| Топик                                                                 | Тип                                                                           | Назначение                                 |
| -------------------------------------------------------------------------- | -------------------------------------------------------------------------------- | ---------------------------------------------------- |
| `image_topic` (по умолчанию `/camera/image_raw/compressed`) | `sensor_msgs/msg/Image` (через image_transport, transport=`compressed`) | Видеопоток с камеры                 |
| `camera_info_topic` (по умолчанию `/camera/camera_info`)    | `sensor_msgs/msg/CameraInfo`                                                   | Внутренние параметры камеры |

### 4.2 Исходящие топики `landing_detector`

| Топик                      | Тип                                      | Назначение                                                                                                      |
| ------------------------------- | ------------------------------------------- | ------------------------------------------------------------------------------------------------------------------------- |
| `/mavros/landing_target/pose` | `geometry_msgs/msg/PoseStamped`           | Основная целевая поза посадочной площадки для контура управления |
| `debug_image`                 | `sensor_msgs/msg/Image` (image_transport) | Отладочная визуализация детекции                                                            |

### 4.3 Входящие топики `precision_lander_node`

| Топик                      | Тип                              | Назначение                                                                         |
| ------------------------------- | ----------------------------------- | -------------------------------------------------------------------------------------------- |
| `/mavros/local_position/pose` | `geometry_msgs/msg/PoseStamped`   | Текущая поза БВС                                                               |
| `/mavros/landing_target/pose` | `geometry_msgs/msg/PoseStamped`   | Поза посадочной цели от `landing_detector`                             |
| `/mavros/state`               | `mavros_msgs/msg/State`           | Статус соединения/arm/mode                                                   |
| `/mavros/extended_state`      | `mavros_msgs/msg/ExtendedState`   | Флаг ON_GROUND и пр.                                                                  |
| `/mavros/mission/waypoints`   | `mavros_msgs/msg/WaypointList`    | Список миссии для логики перехвата перед посадкой |
| `/mavros/mission/reached`     | `mavros_msgs/msg/WaypointReached` | Событие достижения waypoint                                                 |

### 4.4 Исходящие топики `precision_lander_node`

| Топик                            | Тип                             | Назначение                          |
| ------------------------------------- | ---------------------------------- | --------------------------------------------- |
| `/mavros/setpoint_velocity/cmd_vel` | `geometry_msgs/msg/TwistStamped` | Команда скоростей в OFFBOARD |

## 5. Сервисное взаимодействие

Узел `precision_lander_node` использует сервисы MAVROS:

| Сервис           | Тип                          | Назначение                                  |
| ---------------------- | ------------------------------- | ----------------------------------------------------- |
| `/mavros/set_mode`   | `mavros_msgs/srv/SetMode`     | Переключение в `OFFBOARD`              |
| `/mavros/cmd/arming` | `mavros_msgs/srv/CommandBool` | Запрос дизарма после посадки |

Алгоритм:

- при входе в рабочую фазу посадки узел асинхронно запрашивает `OFFBOARD`
- при состоянии `LANDED` и `ON_GROUND` запрашивается дизарм

## 6. TF-взаимодействие

`landing_detector` публикует:

- `camera_frame -> marker_<id>`
- `marker_<id> -> marker_<id>_offset`
- сглаженный `map_frame -> landing_pad_frame` (по умолчанию `map -> landing_pad`, в launch задано `map -> landing_plane`)

`aruco_tracker` публикует:

- `camera_frame -> marker_<id>`

## 7. Параметры, влияющие на информационный обмен

### 7.1 `landing_detector`

| Параметр      | Значение по умолчанию | Влияние                                            |
| --------------------- | ---------------------------------------- | --------------------------------------------------------- |
| `config_path`       | `board.yml`                            | Геометрия маркеров/доски            |
| `image_topic`       | `/camera/image_raw`                    | Источник изображения                   |
| `camera_info_topic` | `/camera/camera_info`                  | Источник intrinsics                               |
| `map_frame`         | `map`                                  | Родительский TF фрейм                    |
| `landing_pad_frame` | `landing_pad`                          | Имя TF цели посадки                         |
| `history_size`      | `1`                                    | Глубина окна сглаживания            |
| `invert`            | `false`                                | Инверсия кадра перед детекцией |

### 7.2 `precision_lander_node`

Ключевые параметры управления обменом и частотой:

- `control_frequency` (в конфигурационном файле 30 Гц)
- пороги переходов: `centering_threshold`, `landing_threshold`, `yaw_threshold`
- ограничения команд: `max_velocity_xy`, `max_velocity_z`, `max_yaw_rate`
- скорости снижения: `landing_speed`, `final_landing_speed`

## 8. Частоты и QoS

### 8.1 `precision_lander_node`

- Главный контур: таймер `1/control_frequency` (конфиг: 30 Гц)
- `mavros_qos`: `BEST_EFFORT`, `KEEP_LAST(10)`, `VOLATILE`
- `sensor_qos` для цели посадки: `RELIABLE`, `KEEP_LAST(1)`

### 8.2 `landing_detector`

- Таймер сглаживания/публикации: каждые 30 мс (≈33 Гц)
- Публикация `/mavros/landing_target/pose` происходит в `smoothAndPublishTF()` при наличии валидного TF

## 9. Логика обмена данными (последовательность)

1. Камера публикует изображение и `camera_info`.
2. `landing_detector` получает intrinsics, затем обрабатывает кадры.
3. При детекции маркеров публикуются TF и сглаженная `PoseStamped` в `/mavros/landing_target/pose`.
4. `precision_lander_node` получает:
   - текущую позу БВС из MAVROS
   - позу цели посадки из `landing_detector`
   - состояние/миссию MAVROS.
5. Узел формирует `TwistStamped` в `/mavros/setpoint_velocity/cmd_vel`.
6. При необходимости вызывает `/mavros/set_mode` и `/mavros/cmd/arming`.

## 10. Условия корректной интеграции

- Топик цели посадки должен быть строго `/mavros/landing_target/pose`.
- Согласовать frame-конвенцию: `map_frame`, `landing_pad_frame`, `base_link`.
- Обеспечить актуальный `camera_info` до старта детекции.
- Для OFFBOARD требуется поток setpoint-команд с достаточной частотой (в проекте поддерживается периодической повторной публикацией последней команды).

## 11. Известные особенности реализации

- В `landing_detector` подписка на `camera_info` отключается после первого успешного сообщения.
- В `precision_lander_node` реализован перехват миссии: при `AUTO.MISSION` и свежей цели возможен переход в `OFFBOARD`.

## 12. Рекомендуемый минимальный контур запуска

1. MAVROS + FCU/PX4.
2. Публикация камеры (`image` + `camera_info`).
3. `ros2 launch precision_landing_ros2 precision_landing.launch.py`.
4. `ros2 launch aruco_tracker landing_detector.launch.py`.

## 13. Порядок действий алгоритма

### 13.1 Сценарий A: старт системы -> OFFBOARD -> посадка

1. Узел `precision_lander_node` запускается в состоянии `INITIALIZING`.
2. После получения подключения к FCU через MAVROS (`/mavros/state.connected = true`) выполняется переход в `IDLE`.
3. В `IDLE` узел ожидает условия старта контура посадки:
   - БВС вооружен (`armed = true`),
   - режим полета: `OFFBOARD` или `AUTO.LAND` или `AUTO.PRECLAND`.
4. При выполнении условий узел переходит в `SEARCHING` и отправляет запрос на `/mavros/set_mode` c `custom_mode = OFFBOARD`.
5. В `SEARCHING` удерживается нулевая скорость и ожидается цель в `/mavros/landing_target/pose`.
6. При получении цели происходит переход в `CENTERING`.
7. В `CENTERING` выполняется PID-выравнивание по X/Y/Z и yaw до порогов:
   - расстояние до цели < `centering_threshold`,
   - ошибка yaw < `yaw_threshold`.
8. После достижения порогов состояние меняется на `DESCENDING`.
9. В `DESCENDING` сохраняется коррекция по X/Y и выполняется снижение; при относительной высоте < 0.4 м переход в `LANDING`.
10. В `LANDING` выполняется финальное вертикальное снижение со скоростью `final_landing_speed` до подтверждения `ON_GROUND` из `/mavros/extended_state`.
11. После `ON_GROUND` узел переходит в `LANDED` и отправляет дизарм через `/mavros/cmd/arming` (`value = false`).
12. После фактического дизарма выполняется reset автомата в безопасное начальное состояние.

### 13.2 Сценарий B: старт в режиме `AUTO.MISSION` -> перехват -> посадка

1. Узел запускается и проходит `INITIALIZING -> IDLE`.
2. БВС летит в `AUTO.MISSION`; узел получает состояние, waypoint-лист и события достижений waypoint.
3. На каждом цикле вызывается `_check_mission_interception()`.
4. Разрешение на перехват включается, если выполнено хотя бы одно условие:
   - следующий waypoint находится в последних трех точках миссии,
   - или следующий waypoint имеет команду `MAV_CMD_NAV_LAND` / `MAV_CMD_NAV_VTOL_LAND`.
5. При разрешении на перехват проверяется цель посадки:
   - `target_pose` присутствует,
   - возраст `target_pose` < 1 секунды.
6. Если цель валидна и свежая, узел инициирует переход в `OFFBOARD` через `/mavros/set_mode`.
7. Далее работа идет по штатной цепочке посадки:
   - `SEARCHING -> CENTERING -> DESCENDING -> LANDING -> LANDED`.
8. Завершение посадки: подтверждение `ON_GROUND`, запрос дизарма, reset в начальное состояние.
