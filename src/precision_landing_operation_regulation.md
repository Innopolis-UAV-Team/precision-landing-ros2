# Регламент работы с модулем точной посадки

## 1. Назначение документа
Настоящий документ устанавливает порядок подключения, конфигурирования и эксплуатации модуля точной посадки БВС в составе ROS 2 системы на базе репозиториев:
- `drone_bringup_ros2`
- `precision-landing-ros2`

Документ обязателен к применению при вводе системы в эксплуатацию, замене камеры, замене посадочного маркера и обслуживании автозапуска.

## 2. Питание и базовые подключения
1. Питание подается через разъем Molex. Допустимое входное напряжение: до 30 В.
2. USB-камера подключается в любой из двух USB-разъемов вычислительного модуля.
3. Свободный UART-кабель подключается к полетному контроллеру PX4 в разъем `TELEM`.
4. Ethernet-порт используется для отладки и мониторинга выполнения посадки (включая Foxglove), а также для SSH-доступа.

## 3. Камера: замена и калибровка
### 3.1 Текущая конфигурация в проекте
В автозапуске используется launch-файл:
- `drone_bringup_ros2/launch/auto_landing.launch.py`

В нем для `usb_cam` сейчас выбран файл параметров:
- `drone_bringup_ros2/config/decxin.yaml`

Ключевые параметры камеры задаются в YAML (`video_device`, `image_width`, `image_height`, `framerate`, `pixel_format`, `camera_info_url`).

### 3.2 Порядок замены камеры
1. Откалибровать новую камеру с помощью ROS-пакета `camera_calibration`.
2. Сохранить файл калибровки в каталог:
   - `drone_bringup_ros2/config`
3. Создать (или скопировать по образцу) YAML-файл параметров `usb_cam` в этом же каталоге.
4. Указать новый YAML-файл в `auto_landing.launch.py` (переменная `camera_config`, блок `config/<имя_файла>.yaml`).
5. Проверить, что в новом YAML корректно задан `camera_info_url` на актуальный calibration-файл.

## 4. Замена ArUco/посадочной схемы
1. При замене маркера(ов) обновить описание в файле:
   - `drone_bringup_ros2/config/board.json`
2. Проверить соответствие:
   - ID маркеров,
   - размеры (`size`),
   - координаты (`x`, `y`),
   - используемый словарь (`type`, например `Aruco_4x4_50`).
3. После изменений перезапустить контур `landing_detector_node`.

Примечание: в текущей конфигурации `auto_landing.launch.py` передает `board.json` в параметр `config_path` узла `landing_detector`.

## 5. Настройка PX4/MAVLink по UART
Для канала к вычислителю необходимо включить MAVLink-поток с высокой скоростью на соответствующем UART порту PX4.

Рекомендуемый файл `extras.txt` (размещается на SD-карте PX4 в каталоге `etc/`):

```bash
mavlink start -d /dev/ttyS3 -b 921600 -m onboard
mavlink stream -d /dev/ttyS3 -s HIGHRES_IMU -r 200
mavlink stream -d /dev/ttyS3 -s ATTITUDE_QUATERNION -r 200
mavlink stream -d /dev/ttyS3 -s GPS_RAW_INT -r 30
mavlink stream -d /dev/ttyS3 -s ALTITUDE -r 30
mavlink stream -d /dev/ttyS3 -s LOCAL_POSITION_NED -r 60
```

Указанный порт `/dev/ttyS3` соответствует `TELEM2` на автопилоте 6C Mini.

Важно:
- порт в PX4 (`extras.txt`) и порт в запуске MAVROS должны соответствовать физическому подключению;
- в текущем `auto_landing.launch.py` MAVROS запускается с `fcu_url:=/dev/ttyS0:921600`.
Если фактически используется `TELEM2 (/dev/ttyS3)`, параметр `fcu_url` должен быть скорректирован.

## 6. Сеть, SSH и мониторинг
1. Ethernet используется для:
   - SSH-доступа,
   - визуального контроля через Foxglove Bridge.
2. Параметры доступа (согласно предоставленным данным):
   - пользователь: `orangepi`
   - хост: `192.168.144.100`
   - пароль: `oragepi`

Рекомендуется проверить пароль и при необходимости обновить на актуальный в эксплуатационной документации.

## 7. Автозапуск системы
Автозапуск реализован user-service:
- `drone_bringup_ros2/drone_bringup.service`
- исполняемый скрипт: `/home/orangepi/start_ros.sh`

Скрипт запускает последовательно:
1. `ros2 launch drone_bringup_ros2 auto_landing.launch.py`
2. `ros2 launch precision_landing_ros2 precision_landing.launch.py`
3. `ros2 launch foxglove_bridge foxglove_bridge_launch.xml`

Управление сервисом:
```bash
systemctl --user status drone_bringup.service
systemctl --user stop drone_bringup.service
systemctl --user start drone_bringup.service
systemctl --user disable drone_bringup.service
systemctl --user enable drone_bringup.service
```

## 8. Порядок работы алгоритма точной посадки
### 8.1 Контур данных (функциональная цепочка)
1. `usb_cam` публикует изображение и `camera_info`.
2. `aruco_tracker/landing_detector_node` детектирует маркеры по `board.json` и публикует цель:
   - `/mavros/landing_target/pose`
3. `precision_landing_ros2/precision_lander_node` получает:
   - `/mavros/local_position/pose`
   - `/mavros/landing_target/pose`
   - `/mavros/state`
   - `/mavros/extended_state`
   - `/mavros/mission/waypoints`
   - `/mavros/mission/reached`
4. Узел управления выдает скорость в:
   - `/mavros/setpoint_velocity/cmd_vel`
5. При необходимости вызывает сервисы:
   - `/mavros/set_mode` (переход в `OFFBOARD`)
   - `/mavros/cmd/arming` (дизарм после посадки)

### 8.2 Сценарий A: старт системы -> OFFBOARD -> посадка
1. После запуска `precision_lander_node` находится в состоянии `INITIALIZING`.
2. После подтверждения подключения MAVROS выполняется переход в `IDLE`.
3. В `IDLE` узел ожидает:
   - `armed = true`
   - режим: `OFFBOARD` или `AUTO.LAND` или `AUTO.PRECLAND`.
4. При выполнении условий переход в `SEARCHING` и асинхронный запрос `OFFBOARD`.
5. После появления цели (`/mavros/landing_target/pose`) переход в `CENTERING`.
6. В `CENTERING` выполняется PID-выравнивание по X/Y/Z и yaw до порогов (`centering_threshold`, `yaw_threshold`).
7. Затем переход в `DESCENDING` и снижение с коррекцией по XY.
8. При достижении малой относительной высоты переход в `LANDING`.
9. После подтверждения `ON_GROUND` переход в `LANDED`, отправка дизарма и reset автомата.

### 8.3 Сценарий B: старт в `AUTO.MISSION` -> перехват миссии -> посадка
1. Система стартует в `INITIALIZING -> IDLE`, БВС выполняет миссию в `AUTO.MISSION`.
2. На каждом цикле проверяется условие перехвата миссии:
   - переход к финальным waypoint, либо
   - следующий waypoint имеет команду `NAV_LAND`/`VTOL_LAND`.
3. Дополнительно проверяется «свежесть» цели посадки (по времени сообщения).
4. При выполнении условий узел инициирует переход в `OFFBOARD`.
5. Далее выполняется штатная цепочка:
   - `SEARCHING -> CENTERING -> DESCENDING -> LANDING -> LANDED`.

## 9. Проверка перед вылетом (чек-лист)
1. Питание и физические подключения проверены.
2. Камера определилась в Linux (`/dev/video*`), параметры в `usb_cam` актуальны.
3. `board.json` соответствует фактическому посадочному полю.
4. MAVROS подключен к правильному UART порту и скорости.
5. Топик `/mavros/landing_target/pose` публикуется стабильно.
6. Топик `/mavros/setpoint_velocity/cmd_vel` появляется после старта алгоритма.
7. Foxglove и SSH доступны по Ethernet.
