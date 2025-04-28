import os

ignore_dirs = {'.venv', '__pycache__', 'logs'}  # 你想忽略的目录

def list_files(base_path, indent=0):
    for name in sorted(os.listdir(base_path)):
        full_path = os.path.join(base_path, name)
        if os.path.isdir(full_path):
            if name in ignore_dirs:
                continue
            print('    ' * indent + f'📁 {name}/')
            list_files(full_path, indent + 1)
        else:
            print('    ' * indent + f'📄 {name}')

if __name__ == "__main__":
    project_path = '.'  # 当前目录，可以换成你的项目路径
    list_files(project_path)
