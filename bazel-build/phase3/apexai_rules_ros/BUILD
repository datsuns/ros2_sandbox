load("@rules_python//python:py_runtime.bzl", "py_runtime")
load("@rules_python//python:py_runtime_pair.bzl", "py_runtime_pair")

py_runtime(
    name = "system_python_runtime",
    interpreter_path = "/usr/bin/python3",
    python_version = "PY3",
)

py_runtime_pair(
    name = "system_python_runtime_pair",
    py3_runtime = ":system_python_runtime",
)

toolchain(
    name = "system_python_toolchain",
    toolchain = ":system_python_runtime_pair",
    toolchain_type = "@rules_python//python:toolchain_type",
)
