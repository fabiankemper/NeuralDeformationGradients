from huggingface_hub import HfApi, create_repo

REPO_ID = "fabiankemper/ndg" 
FOLDER_TO_UPLOAD = "data/models"

api = HfApi()

def main():
    print(f"Ensuring repository '{REPO_ID}' exists...")
    create_repo(repo_id=REPO_ID, repo_type="model", exist_ok=True, private=False)

    print(f"Uploading contents of '{FOLDER_TO_UPLOAD}' to '{REPO_ID}'...")
    
    # Upload the entire folder
    api.upload_folder(
        folder_path=FOLDER_TO_UPLOAD,
        repo_id=REPO_ID,
        repo_type="model",
        path_in_repo="data/models", 
    )
    
    print("Upload complete! 🚀")

if __name__ == "__main__":
    main()
