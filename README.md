# DSSLRobot  gh pages

DSSLRobot  gh pages is an open technical documentation platform for the DSSL Robot team, aimed at helping team members better learn and share knowledge related to robot development and provides the detailed deployment and usage for our robot platform and relavant alogrithms.
DSSLRobot  gh pages is built with [Docusaurus 2](https://docusaurus.io/).

## Contributing
We welcome contributions to DSSLRobot  gh pages! If you would like to contribute, please choose one of the methods and follow the steps below:

### Method 1
1. `fork` the repository to your own GitHub account, for example `glaciercoder/dsslrobot.github.io`.
2. `clone` your repository and write your article (following the [project structure](https://DsslRobot.github.io/tinkerdocs/docs/Intro/project_structure)).
3. `commit` your changes and `push` them to your repository.
4. Create a `pull request` to the `main` branch of the `DsslRobot/DsslRobot.github.io` repository.
5. Wait for the `github actions` to build and deploy your changes to the site. Once the build is successful, your changes will be live on the site.
6. If the build fails, you will need to fix the issues and push the changes to your repository. The build will automatically be triggered again.

For more information, you can take a look at [how-to-fork-a-github-repository](https://www.freecodecamp.org/chinese/news/how-to-fork-a-github-repository/).

### Method 2

1. Directly clone the repository to your local computer.
2. Create a new branch: 
   ```sh
   git checkout -b <branch_name>
   ```
3. Write your article in the new branch 
4. `commit` your changes and `push` them to remote repository.
5. Go to the remote repository and create a pull request.
6. Solve the confict and merge your own branch to `main`
7. Wait for the `github actions` to build and deploy your changes to the site. Once the build is successful, your changes will be live on the site.
8. If the build fails, you will need to fix the issues and push the changes to your repository. The build will automatically be triggered again.
   
## Commands

If you want to build and deploy the site locally, you can use the following commands:

### Installation

clone this repository, make sure you have npm installed and `node.js` version is right. [Installation](https://nodejs.org/en/download/package-manager)


### Local Development

```
npm run start
```

This command starts a local development server and opens up a browser window. Most changes are reflected live without having to restart the server.

### Build

```
npm run  build
```

This command generates static content into the `build` directory and can be served using any static contents hosting service.

### Testing your Build Locally

```
npm run  serve
```


### Deployment

We have setup the github action for automatic deployment. If you have the need to manually deploy, follow the steps below:

#### For Bash
Using SSH:

```
$ USE_SSH=true yarn deploy
```

Not using SSH:

```
$ GIT_USER=<Your GitHub username> yarn deploy
```

#### For Windows

Using SSH:

```
cmd /C "set "USE_SSH=true" && yarn deploy"
```

Not using SSH:

```
cmd /C "set "GIT_USER=<GITHUB_USERNAME>" && yarn deploy"
```
If you are using GitHub pages for hosting, this command is a convenient way to build the website and push to the `gh-pages` branch.

