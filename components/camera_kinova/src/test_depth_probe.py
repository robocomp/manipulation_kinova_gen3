#!/usr/bin/env python3
import sys
import os
import time

sys.path.insert(0, os.path.dirname(__file__))

from kortex_api.autogen.client_stubs.VisionConfigClientRpc import VisionConfigClient
from kortex_api.autogen.client_stubs.DeviceManagerClientRpc import DeviceManagerClient
from kortex_api.autogen.messages import DeviceConfig_pb2, VisionConfig_pb2

import utilities


def get_vision_device_id(device_manager):
    all_devices_info = device_manager.ReadAllDevices()
    vision_handles = [hd for hd in all_devices_info.device_handle if hd.device_type == DeviceConfig_pb2.VISION]
    if len(vision_handles) == 0:
        print("No vision device registered")
        return 0
    if len(vision_handles) > 1:
        print(f"More than one vision device registered: {len(vision_handles)}")
        return 0
    handle = vision_handles[0]
    print(f"Vision module found, device id: {handle.device_identifier}")
    return handle.device_identifier


def main():
    args = utilities.parseConnectionArguments()

    try:
        with utilities.DeviceConnection.createTcpConnection(args) as router:
            device_manager = DeviceManagerClient(router)
            vision_config = VisionConfigClient(router)
            vision_device_id = get_vision_device_id(device_manager)
            if vision_device_id == 0:
                return

            option_identifier = VisionConfig_pb2.OptionIdentifier()
            option_identifier.sensor = VisionConfig_pb2.SENSOR_DEPTH
            print("\nChecking depth options...")
            for option in VisionConfig_pb2.Option.values()[1:]:
                try:
                    option_identifier.option = option
                    info = vision_config.GetOptionInformation(option_identifier, vision_device_id)
                    if info.sensor == VisionConfig_pb2.SENSOR_DEPTH and int(info.supported) == 1:
                        print(
                            f"option={VisionConfig_pb2.Option.Name(option)} supported=1 read_only={info.read_only} "
                            f"min={info.minimum} max={info.maximum} step={info.step} default={info.default_value}"
                        )
                except Exception:
                    pass
    except Exception as exc:
        print(f"Probe failed: {exc}")


if __name__ == '__main__':
    main()
