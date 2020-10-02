# Contributing to VSS Simulation 🚀

Para a versão em PT-BR :brazil: desse documento, [veja aqui](link).

First of all, thanks for your interest! We really appreciate the participation of the community! ✨

Here we describe a set of guidelines when contributing to the VSS simulation. These are not strict rules, so use your best judgment, and in case you have questions, contact us on contato@thunderatz.org.

## How can I contribute?

### Issues
Most contributions are made with [GitHub Issues](https://guides.github.com/features/issues/). They will be primarily used for

1. Trackking bugs - `bug`
2. Suggesting Enhancements - `enhancement`
3. Improving documentation - `documentation`

For each of these, there is a specific [label](https://docs.github.com/en/enterprise/2.17/user/github/managing-your-work-on-github/applying-labels-to-issues-and-pull-requests). We strongly recommend that every issue created has at least one of the three labels described above. 

But before creating an issue, first check if there is already one with the same subject. You can search and filter by labels, for example, [here](https://github.com/ThundeRatz/vss_simulation/labels/bug) we only see the issues that are active and have the `bug` label.

#### Reporting Bugs 🐛
- Use a clear title
- Specify the version of the package
- Specify OS, VM (if applicable), packages installed, and other configurations that may be usefull
- Describe the steps to reproduce the bug
- Describe the observed and the expected behavior
- Include screenshots, gifs, any type of reference that helps to explain the problem

#### Suggesting Enhancements ✨
- Use a clear title
- Describe the suggestion step-by-step
- Describe the expected behavior after implementing this idea
- Explain why this new feature or update can be usefull

#### Imporoving documentation 📝
- Use a clear title
- Specify the files that need to be documented
- Explain your suggestion and why would it be better/clearer

### Pull Requests
If you want to contribute with code to the project, search for an open issue and start developing. When you are ready, open a Pull Request and we will review it.

A few recomendations:

- Describe exactly what you've done, always as clear as possible
- Link the applicable issue in your Pull Request (if there is none, please, create a new one)
- Make sure that you are following the [Syleguide](#Styleguide)

## Styleguide 💄
The code and structure must follow the the [ROS Use Patterns](http://wiki.ros.org/ROS/Patterns) and [ROS Best Practices](http://wiki.ros.org/BestPractices).

### Python
Python code should follow the [ROS Python](http://wiki.ros.org/PyStyleGuide) and the [PEP 8](https://www.python.org/dev/peps/pep-0008/) style guides.

### Git commit messages
- Use the present tense ("Add feature" not "Added feature")
- Use the imperative mood ("Move cursor to..." not "Moves cursor to...")
- It is strongly recommended to start a commit message with a related emoji
  - 📝 `:memo:` for documentation
  - 🐛 `:bug:` for bug issues
  - 🚑 `:ambulance:` for critical fixes
  - 🎨 `:art:` for imporoving structure
  - ✨ `:sparkles:` for new features
  
  For more examples, see [this reference](https://gitmoji.carloscuesta.me/).

### Git workflow
The project workflow is based on [Git Flow](https://nvie.com/posts/a-successful-git-branching-model/).

### Documentation
The documentation is generated with [Doxygen](https://www.doxygen.nl/index.html) and should follow its documenting manual.
