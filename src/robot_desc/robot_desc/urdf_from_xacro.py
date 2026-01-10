import subprocess

def get_urdf_from_xacro(xacro_file: str) -> str:
    result = subprocess.run(
        ["xacro", xacro_file],  # 修正命令名拼写
        stdout=subprocess.PIPE,
        text=True,
        check=True
    )
    return result.stdout