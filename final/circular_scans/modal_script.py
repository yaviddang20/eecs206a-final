import modal
import subprocess

app = modal.App("instasplat")
custom_image = modal.Image.from_registry("davidyang20/instasplat").add_local_dir("run30", remote_path="/run/images")
# datascience_image = (
#     modal.Image.debian_slim()
#     .uv_pip_install("pandas==2.2.0", "numpy")
# )


# @app.function(image=datascience_image)
# def my_function():
#     import pandas as pd
#     import numpy as np
#     print("Pandas version:", pd.__version__)


@app.function(image=custom_image)
def test():
    result = subprocess.run(["python init_geo.py --source_path /run --model_path /run30/out --n_views 14"], check=True, capture_output=True, text=True)
    # return result.stdout.strip()
    # import numpy

    # print(numpy.__version__)
    return result.stdout.strip()


@app.local_entrypoint()
def main():
    print("the current directory is", test.remote())
    # my_function

