"""ROS namespace layout of the Lookie drivers under ``<namespace>/driver``."""

DRIVER_NAMESPACE = 'driver'  # relative to the robot namespace
HEAD_MODULE = 'head'
TORSO_NAMESPACE = 'torso'
ARM_SIDES = ('left_arm', 'right_arm')
CHASSIS_NAMESPACE = 'chassis'  # not a bridge module; kstone talks to it directly


def module_namespaces(torso_key: str) -> dict:
    """Map bridge module name to its namespace under the driver root."""
    return {HEAD_MODULE: HEAD_MODULE, torso_key: TORSO_NAMESPACE,
            **{side: side for side in ARM_SIDES}}


def validate_namespace(namespace: str) -> str:
    """Return the robot namespace without surrounding slashes; raise ValueError if invalid."""
    ns = str(namespace).strip().strip('/')
    if not ns or any(not part or not part.replace('_', '').isalnum() or part[0].isdigit()
                     for part in ns.split('/')):
        raise ValueError(f'invalid namespace {namespace!r}')
    return ns


def driver_namespace(namespace: str) -> str:
    return f'{namespace}/{DRIVER_NAMESPACE}'
