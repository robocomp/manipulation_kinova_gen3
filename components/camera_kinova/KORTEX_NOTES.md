# Notas de Kortex para la cámara del robot

## 1. API instalada

- Ruta de instalación local: `/home/robolab/.local/lib/python3.10/site-packages/kortex_api`
- Paquete Python detectado: `kortex_api 2.6.0.post3`
- El componente puede usar los clientes generados para:
  - `VisionConfigClient`
  - `DeviceConfigClient`
  - `DeviceManagerClient`

## 2. Servicios relevantes para vision

### VisionConfigClient

Métodos útiles:
- `GetOptionInformation(option_identifier, device_id)`
- `GetOptionValue(option_identifier, device_id)`
- `SetOptionValue(option_value, device_id)`
- `GetSensorSettings(sensor_identifier, device_id)`
- `SetSensorSettings(sensor_settings, device_id)`
- `GetIntrinsicParameters(sensor_identifier, device_id)`
- `GetExtrinsicParameters(device_id)`

### DeviceManagerClient

Sirve para localizar dispositivos registrados en el robot, incluyendo el módulo de visión.

### DeviceConfigClient

Permite reiniciar el módulo de visión si hace falta.

## 3. Tipos importantes

### Sensores

- `SENSOR_COLOR`
- `SENSOR_DEPTH`

### Opciones de vision

El enum `Option` expone muchas opciones de configuración, por ejemplo:
- `OPTION_BACKLIGHT_COMPENSATION`
- `OPTION_BRIGHTNESS`
- `OPTION_CONTRAST`
- `OPTION_EXPOSURE`
- `OPTION_GAIN`
- `OPTION_GAMMA`
- etc.

### Resoluciones y tasas

- `RESOLUTION_320x240`
- `RESOLUTION_424x240`
- `RESOLUTION_480x270`
- `RESOLUTION_640x480`
- `RESOLUTION_1280x720`
- `RESOLUTION_1920x1080`

- `FRAMERATE_6_FPS`
- `FRAMERATE_15_FPS`
- `FRAMERATE_30_FPS`

## 4. Uso habitual para inspeccionar depth

El flujo recomendado es:

1. Conectar al robot vía `DeviceConnection.createTcpConnection(...)`.
2. Crear:
   - `DeviceManagerClient(router)`
   - `DeviceConfigClient(router)`
   - `VisionConfigClient(router)`
3. Pedir el `vision_device_id` consultando los dispositivos registrados.
4. Para el sensor depth, usar:
   - `OptionIdentifier(sensor=SENSOR_DEPTH, option=...)`
   - `GetOptionInformation(...)`
   - `GetOptionValue(...)`
5. Si se necesita, ajustar con `SetOptionValue(...)`.

## 5. Ejemplo de referencia usado

Se ha tomado como referencia el ejemplo adjunto:
- `04-vision_sensor_options.py`

Su lógica principal es:
- localizar el dispositivo de visión,
- listar las opciones soportadas por color/depth,
- leer sus valores,
- y probar a cambiarlos.

## 6. Qué nos dice esto para este componente

Para la cámara de RoboComp, esto sugiere que:
- el depth no debe tratarse solo como un stream RTSP más,
- sino como un sensor de visión del robot que puede tener opciones y estados internos,
- y que conviene consultar su disponibilidad con la API antes de asumir que el endpoint RTSP es usable.

## 7. Nota práctica

Si el `kortex_api` está instalado correctamente, el componente puede intentar:
- consultar el sensor depth con `VisionConfigClient`,
- y usar esa información para decidir si el stream de depth está habilitado o si hay que dejarlo como no disponible.
