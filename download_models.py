from pathlib import Path
from huggingface_hub import snapshot_download

REPO_URL = "fabiankemper/ndg"
DEST_DIR = Path(".")

def main():
    print(f"Downloading model files from '{REPO_URL}'...")
    DEST_DIR.mkdir(parents=True, exist_ok=True)

    snapshot_download(repo_id=REPO_URL, local_dir=str(DEST_DIR))
    print(f"Model files downloaded to '{DEST_DIR}'.")

if __name__ == "__main__":
    main()
