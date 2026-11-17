# How to put this repo on GitHub

Do this once. Everything else happens in VS Code afterwards.

## 1. Tell git who you are (once per computer)
```
git config --global user.name "Sanjit"
git config --global user.email "sanjit0907@gmail.com"
```
Havinesh runs the same two lines on his own laptop with his own name and email.

## 2. Make an empty repository on github.com
- New repository -> name: `pde4446-robot`
- Visibility: Private
- Do NOT tick "Add a README" (this folder already has one)
- Create repository
- Settings -> Collaborators -> add Havinesh

## 3. Connect this folder and upload
In the VS Code terminal, inside this folder:
```
git remote add origin https://github.com/YOUR-USERNAME/pde4446-robot.git
git push -u origin main
```
GitHub will ask you to sign in the first time.

## 4. Havinesh gets a copy
```
git clone https://github.com/YOUR-USERNAME/pde4446-robot.git
```

## Daily use in VS Code
1. Edit and save a file.
2. Source Control panel (branch icon, left bar) -> click `+` on the changed file.
3. Type a message, e.g. `Tune KP to 0.3, left wheel now tracks`.
4. Click Commit, then Sync Changes.
5.
