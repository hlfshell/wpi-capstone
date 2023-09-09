# Quickstart

This document will act as a quickstart guide for setting up your environment for the project and getting started in committing code to it. It will list environment and project application choices, settings, and versions where appropriate.

# Required software

* [ ] bash or equivalent terminal - we are assuming that you're using some equivalent terminal tool to bash - zsh is also acceptable (default for macOS). Windows machines need a bash terminal emulator such as [Git Bash](https://git-scm.com/downloads) or [Cygwin](https://www.cygwin.com/).
* [ ] docker set up on your machine and running without needing root
* [ ] virtualbox setup on your machine for VMs
* [ ] [vagrant](https://developer.hashicorp.com/vagrant/downloads) installed and set up on your machine's `PATH`
* [ ] [just](https://github.com/casey/just) installed and set up on your machine's `PATH`
* [ ] A virtual environment manager for Python (such as `pyenv` or `virtualenv`)
* [ ] If you are a macOS user, you need to `brew install coreutils` for the `realpath` command.

# Python setup

We'll be using Python `3.10.9` as our chosen version.

All utilized modules will be stored in `requirements.txt`, generated via running `pip freeze > requirements.txt` in your cli with the virtual environment enabled. To install this file, run `pip install -r requirements.txt` in your cli with the virtual environment enabled. Note that these requirements can change/expand over time, so you may need to run this command again to install new modules.

After installing your environment, ensure that you have `flake8`, `black`, and `isort` installed within your environment and executable from your shell. All python `just` commands (such as `lint`) will assume that you are operating them *within* your virtual environment.

If you want to check your code for issues, utilize `just lint` - it will report to you issues that exist within the formatting of your code. Use `just fix` to fix your imports and code formatting; an additional lint is automatically ran afterwards to report issues that the linters won't fix for you, but might still be noteworthy (such as unused imports and variables, as you may be in the middle of a work in progress).

You can also configure your IDE to do this on file save, which I recommend.

Thus the steps to follow are:

1. Clone the Repository

Open Git Bash and navigate to your specified directory where you want to store your copy  of the github repo :
```
cd "your/project/file/location"
git clone https://github.com/hlfshell/wpi-capstone.git
```

2. Navigate into Project Folder

```
cd wpi-capstone
```

3. Virtual Environment Setup
Create a virtual environment in the current folder:
```
python -m venv .venv
source .venv/Scripts/activate
```

or

```
pyenv virtualenv 3.10.9 wpi-capstone
pyenv shell wpi-capstone
```

4. Install Required Packages

```
pip install -r requirements.txt
```

# Working with Vagrant

We'll be using Vagrant to manage the virtual machine environment for this project. While this is more taxing than running the Docker image, it allows easier usage of GUI applications (the steps needed to get Docker to run the GUI applications get extremely annoying for non Linux teammates).

The `ros_ws` folder is synced with the VM, so any changes on either host or guest machine will be reflected amongst both. This means you can work on your host machine using the tooling you want.

We have a number of `just` commands to make working with the VM easier:

* `just vm-start` will start the VM (equivalent to `vagrant up`). If this is the first time that you're running the VM, it'll take a bit to download and install all tools. Note that since we're installing the desktop GUI for the first time, you will need to restart the VM to get the virtualbox window to show Gnome.

* `just vm-stop` - stops the VM

* `just vm-rebuild` - destroys and rebuilds the VM. If you have issues that are irreconcilable, have experimented too heavily with changes to the VM and want to start fresh, or if we've made major changes elsewhere in the project, doing this can reset your environment to be in line with everyone else's.

* `just vm-bash` - this will create a bash shell ssh'ed into the VM, so you can run command line code

# Working with Docker

We'll be using Docker for the final product due to its lower resource usage and its better compatibility with nvidia drivers for our deep learning tools. The image will be generally referred to as `capstone-ros`.

The `ros_ws` folder is synced between host and guest machines.

We have a number of `just` commands to make working with Docker easier:

* `just docker-build` will build the docker image. If it already exists this will fail. If you've never built the image before, it may take some time, but ill be generally quicker on future builds.

* `just docker-rebuild` will destroy and then rebuild the image. Note that this is still using cached layers, so shouldn't take long.

* `just docker-halt` will stop the container from running (if it is running)

* `just docker-ros` will check to see if the container `capstone-ros` is running, and if not, will start a core ROS container. If the container is already running, it will do nothing.

* `just docker-bash` will create a bash shell connected to the `capstone-ros` container. If the container is not running, it will run it first.

* `just docker-vs-code` - this will start the `docker-ros` shell if it is not already running. It will then start a VS code tunnel (See the VS Code Tunnel section).

# VS Code Tunnel

So we have a problem - ROS2 installs itself in a way that is unfriendly to multiple environments/versions running in parallel - which is bad for a dev machine. We want the ability to switch versions easily and not have them interact with each other. To solve this, we created the VM's and containers to isolate the environment. But this introduces a new problem.

We're use to having our tooling be context-aware of our code - if we're programming in C or Python we want our includes to be detected and auto-suggested in our IDEs, while also ensuring that we can click to follow through into imported objects so we can observe their interfaces, what they're doing under the hood, etc.

Our host machines, however, don't have the ROS2 environment configured as per our isolation preference. Worse, ROS2 doesn't provide any Python libraries or anything outside of installing the whole environment, defeating the goal of our virtualenv isolation.

The solution? VS Code Tunnels for us VS Code users. This runs a VS Code server in a docker image with the environment setup, and can either be accessed as a website, or pass all functionality into the VS Code in your host environment.

To set this up:

1. Ensure that you have your Github account linked to your VS Code, and you have sync enabled (for your extensions, settings, etc)

2. Run `just docker-vs-code` to start the VS Code server in the docker container.

3. Follow the link provided in the terminal to authorize the VS Code server to run connected to your Github account.

At this point, you may click on the URL provided to run VS Code in a browser. If you want to use your own host machine's VS Code, then instead click the bottom left hand corner where a small connection symbol in blue sits. It will pop up a menu, where you can specify to connect to a remote tunnel. Select that option; your new tunnel should be listed. Click this, and the window will reload. Note that the VS Code server will start pulling down your extensions and settings, and may take a moment to load all of your tooling.

We sync the `.code-server` folder in the background in order to prevent this server from being marked as a new server each time, and maintain extensions and settings sync for quicker restarts.

# Code Commit Pattern

We'll be using a standard git flow for this project, with a particular exception - no `dev`/`stage`/`main` version separation due to the nature of the project. `main` will be the primary branch that we will work to maintain as stable as possible so that people can work off of it and run demos/tests.

This is important, as we are a group of 4 that will be working on multiple aspects of our robotic system, likely simultaneously. We want to ensure that people can spend their time working on the code they want to be working on, and not constantly fixing merge conflicts as we step on each other's toes.

To modify code or make changes, please follow the following steps:

1. Create a new branch off of `main` with the following naming convention: `feature/<feature-name>` or `fix/<bugfix-name>`, `docs/<docs-name>`, `chore/<chore-name>`, etc. Branch names should be descriptive, ie `feature/nav2-launch` and not `feature/mikes-changes-1`

2. Commit early and often. Commits should be contextually small bite sized pieces that are easy to understand and review together. We do this because of the existence of `cherry-pick`, which allows us to pick and choose what features from a given branch we may want to pull out later. Large commits simply using `-a` are unhelpful and muddy this process significantly.

3. When your code is ready, ensure that you are ready to merge it into `main` by checking out `main`, pulling any possible changes merged in the meantime, and then merging this back up into your branch. This will ensure that you have the latest code and that your code will merge cleanly into `main`.

4. Create a pull request in GitHub with your code changes. Ensure that you have a descriptive title and description of what your code changes are doing. Your code changes presented tell us the "how" - the pull request tells us the "why".

These pull requests can be merged with 1 approving review. The pull requests will run linting on code to maintain code quality.