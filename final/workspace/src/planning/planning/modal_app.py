import modal
import subprocess

app = modal.App("gs-2m")

custom_image = modal.Image.from_dockerfile("Dockerfile", gpu="A100")


@app.function(image=custom_image, gpu="A100")
def test():
    cmd = """
    eval "$(conda shell.bash hook)"
    conda activate gs2m
    wget -qO- "https://limewire.com/d/L6YHt#xJ9L5kNZ5s" | unzip -
    python scripts/run_tnt.py 
    """

    result = subprocess.run(["bash", "-c", cmd], capture_output=True, text=True)

    return result.stdout.strip(), result.stderr.strip()

@app.local_entrypoint()
def main():
    print("message from the container", test.remote())


