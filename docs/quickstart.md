# Quickstart

This document will act as a quickstart guide for setting up your environment for the project and getting started in committing code to it. It will list environment and project application choices, settings, and versions where appropriate.

# Required software

* [ ] bash or equivalent terminal - we are assuming that you're using some equivalent terminal tool to bash - zsh is also acceptable (default for macOS). Windows machines need a bash terminal emulator such as [Git Bash](https://git-scm.com/downloads) or [Cygwin](https://www.cygwin.com/).
* [ ] docker set up on your machine and running without needing root
* [ ] virtualbox setup on your machine for VMs
* [ ] [vagrant](https://developer.hashicorp.com/vagrant/downloads) installed and set up on your machine's `PATH`
* [ ] [just](https://github.com/casey/just) installed and set up on your machine's `PATH`
* [ ] A virtual environment manager for Python (such as `pyenv` or `virtualenv`)

# Python setup

We'll be using Python `3.10.9` as our chosen version.

All utilized modules will be stored in `requirements.txt`, generated via running `pip freeze > requirements.txt` in your cli with the virtual environment enabled. To install this file, run `pip install -r requirements.txt` in your cli with the virtual environment enabled. Note that these requirements can change/expand over time, so you may need to run this command again to install new modules.

After installing your environment, ensure that you have `flake8`, `black`, and `isort` installed within your environment and executable from your shell. All python `just` commands (such as `lint`) will assume that you are operating them *within* your virtual environment.

If you want to check your code for issues, utilize `just lint` - it will report to you issues that exist within the formatting of your code. Use `just fix` to fix your imports and code formatting; an additional lint is automatically ran afterwards to report issues that the linters won't fix for you, but might still be noteworthy (such as unused imports and variables, as you may be in the middle of a work in progress).

You can also configure your IDE to do this on file save, which I recommend.

# Code Commit Pattern

We'll be using a standard git flow for this project, with a particular exception - no `dev`/`stage`/`main` version separation due to the nature of the project. `main` will be the primary branch that we will work to maintain as stable as possible so that people can work off of it and run demos/tests.

This is important, as we are a group of 4 that will be working on multiple aspects of our robotic system, likely simultaneously. We want to ensure that people can spend their time working on the code they want to be working on, and not constantly fixing merge conflicts as we step on each other's toes.

To modify code or make changes, please follow the following steps:

1. Create a new branch off of `main` with the following naming convention: `feature/<feature-name>` or `fix/<bugfix-name>`, `docs/<docs-name>`, `chore/<chore-name>`, etc. Branch names should be descriptive, ie `feature/nav2-launch` and not `feature/mikes-changes-1`

2. Commit early and often. Commits should be contextually small bite sized pieces that are easy to understand and review together. We do this because of the existence of `cherry-pick`, which allows us to pick and choose what features from a given branch we may want to pull out later. Large commits simply using `-a` are unhelpful and muddy this process significantly.

3. When your code is ready, ensure that you are ready to merge it into `main` by checking out `main`, pulling any possible changes merged in the meantime, and then merging this back up into your branch. This will ensure that you have the latest code and that your code will merge cleanly into `main`.

4. Create a pull request in GitHub with your code changes. Ensure that you have a descriptive title and description of what your code changes are doing. Your code changes presented tell us the "how" - the pull request tells us the "why".

These pull requests can be merged with 1 approving review. The pull requests will run linting on code to maintain code quality.