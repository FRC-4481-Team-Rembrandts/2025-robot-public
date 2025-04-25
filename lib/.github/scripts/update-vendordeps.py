import os
import json
import requests
from packaging import version
import subprocess

VENDORDEP_DIR = "./vendordeps"

for file in os.listdir(VENDORDEP_DIR):
    if not file.endswith(".json"):
        continue

    file_path = os.path.join(VENDORDEP_DIR, file)

    with open(file_path, "r") as f:
        local_data = json.load(f)

    json_url = local_data.get("jsonUrl")

    if not json_url:
        print(f"No 'jsonUrl' key found in {file}")
        continue

    response = requests.get(json_url)

    if response.status_code != 200:
        print(f"Failed to fetch {json_url}")
        continue

    remote_data = response.json()
    local_version = local_data.get("version")
    remote_version = remote_data.get("version")
    remote_file_name = remote_data.get("fileName")
    vendordep_name = remote_data.get("name")

    new_file_path = os.path.join(VENDORDEP_DIR, remote_file_name)

    if version.parse(remote_version) <= version.parse(local_version):
        print(f"No updates needed for {vendordep_name}.")
        continue

    branch_name = f"update-{vendordep_name}-{remote_version}"
    print(f"Updating {vendordep_name}: {local_version} -> {remote_version} on branch {branch_name}")

    # Create branch
    subprocess.run(["git", "checkout", "-b", branch_name])

    # Update file
    with open(file_path, "w") as f:
        json.dump(remote_data, f, indent=2)

    # Rename the file if necessary
    if remote_file_name != file:
        print(f"Renaming file: {file} -> {remote_file_name}")
        os.rename(file_path, new_file_path)

    # Commit changes
    subprocess.run(["git", "add", VENDORDEP_DIR])
    commit_message = f"Bump {vendordep_name} from {local_version} to {remote_version}"
    subprocess.run(["git", "commit", "-m", commit_message])

    # Push branch
    subprocess.run(["git", "push", "-u", "origin", branch_name])

    # Create Pull Request
    pr_body = f"### Automated update for {vendordep_name}\n- **Old version:** {local_version}\n- **New version:** {remote_version}"
    subprocess.run([
        "gh", "pr", "create",
        "--title", commit_message,
        "--body", pr_body,
        "--head", branch_name,
        "--base", "main"
    ])

    # Return to main branch after PR
    subprocess.run(["git", "checkout", "main"])
    subprocess.run(["git", "branch", "-D", branch_name])