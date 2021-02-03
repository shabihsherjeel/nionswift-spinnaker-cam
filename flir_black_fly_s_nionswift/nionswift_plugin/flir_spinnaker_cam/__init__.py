from nion.utils import Registry


from . import CameraDeviceSpinnaker

class FlirSpinnakerCamExtension:

    # required for Swift to recognize this as an extension class.
    extension_id = "flir.spinnakr.cam"

    def __init__(self, api_broker):
        # grab the api object.
        api = api_broker.get_api(version='~1.0', ui_version='~1.0')
        # api can be used to access Nion Swift...

    def close(self):
        pass  # perform any shutdown activities

def run():
    # camera = CameraDeviceSpinnaker.CameraDeviceSpinnaker()
    # logging.info(camera.camera_id)
    camera_device = CameraDeviceSpinnaker.CameraSpinnaker("spinnaker_camera_black_fly_s", "cmos", "Microscope")
    if not hasattr(camera_device, 'camera_id'):
        return
    camera_device.camera_panel_type = "camera_panel_type"

    camera_settings = CameraDeviceSpinnaker.CameraSettings("spinnaker_camera")
    Registry.register_component(CameraDeviceSpinnaker.CameraModule(camera_device, camera_settings), "camera_module")
