import subprocess
import json
import sys

def get_collabs():
    # 1. Get the list of usernames
    cmd = ["gh", "api", "repos/:owner/:repo/collaborators", "--jq", ".[].login"]
    result = subprocess.run(cmd, capture_output=True, text=True)
    if result.returncode != 0:
        print("Error: Make sure you are in a git repo and logged into 'gh'")
        return []
    return result.stdout.strip().split('\n')

def get_user_details(username):
    # 2. Get the specific name and email for that user
    cmd = ["gh", "api", f"users/{username}"]
    result = subprocess.run(cmd, capture_output=True, text=True)
    data = json.loads(result.stdout)
    
    # GitHub doesn't always show email if it's private, 
    # so we fall back to username@users.noreply.github.com
    name = data.get('name') or username
    email = data.get('email') or f"{username}@users.noreply.github.com"
    return name, email

def main():
    usernames = get_collabs()
    if not usernames: return

    print("\n--- GITHUB COLLABORATOR LOGIN ---")
    for i, user in enumerate(usernames, 1):
        print(f"{i}) {user}")

    choice = input("\nSelect your username number: ")
    try:
        selected_username = usernames[int(choice) - 1]
        print(f"Fetching profile for {selected_username}...")
        
        real_name, email = get_user_details(selected_username)

        # 3. Set the local config
        subprocess.run(["git", "config", "--local", "user.name", real_name])
        subprocess.run(["git", "config", "--local", "user.email", email])

        print(f"\nSUCCESS!")
        print(f"Identity set to: {real_name} <{email}>")
    except (ValueError, IndexError):
        print("Invalid selection.")

if __name__ == "__main__":
    main()

