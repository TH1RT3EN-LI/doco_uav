from launch.substitutions import PythonExpression


def normalized_namespace(namespace):
    return PythonExpression(
        [
            "('/' + '",
            namespace,
            "'.strip('/')) if '",
            namespace,
            "'.strip('/') else '/'",
        ]
    )


def namespaced_path(namespace, suffix: str):
    normalized_suffix = suffix if suffix.startswith("/") else f"/{suffix}"
    return PythonExpression(
        [
            "('/' + '",
            namespace,
            "'.strip('/') + '",
            normalized_suffix,
            "') if '",
            namespace,
            "'.strip('/') else '",
            normalized_suffix,
            "'",
        ]
    )


def conditional_namespaced_path(enabled, namespace, suffix: str):
    normalized_suffix = suffix if suffix.startswith("/") else f"/{suffix}"
    return PythonExpression(
        [
            "'' if '",
            enabled,
            "'.lower() != 'true' else (('/' + '",
            namespace,
            "'.strip('/') + '",
            normalized_suffix,
            "') if '",
            namespace,
            "'.strip('/') else '",
            normalized_suffix,
            "')",
        ]
    )
