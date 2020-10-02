# Contribuindo com VSS Simulation 🚀

Primeiramente, obrigado pelo interesse! Nós agradecemos muito pela participação da comunidade nesse projeto! ✨

Esse documento descreve um conjunto de diretrizes para contribuir com esse projeto. Não são regras estritas, então sempre vale o bom senso, e, em caso de dúvidas, nos contate em comp@thunderatz.org.

## Como eu posso contribuir?

### Issues
A maior parte das contribuições é feita por meio de [GitHub Issues](https://guides.github.com/features/issues/). Elas serão usadas principalmente para

1. Identificar bugs - `bug`
2. Sugerir melhorias e recursos - `enhancement`
3. Melhorar a documentação - `documentation`

Para cada um desses itens, existe um [label específico](https://docs.github.com/en/enterprise/2.17/user/github/managing-your-work-on-github/applying-labels-to-issues-and-pull-requests). Nós recomendamos fortemente que todas as issues criadas contenham pelo menos um desses labels descritos a cima.

Mas, antes de criar uma nova issue, é importante primeiro checar se já não existe uma com o mesmo assunto. Você pode filtrar por labels, por exemplo, [aqui](https://github.com/ThundeRatz/vss_simulation/labels/bug) serão mostradas apenas as issues ativas com o label `bug`.

#### Identificando bugs 🐛
- Use títulos claros
- Especifique a versão do pacote
- Especifique o SO, MV (se aplicável), pacotes instalados, e outras configurações que podem ser úteis.
- Descreva os passos para reproduzir o bug encontrado
- Descreva o comportamento observado e o esperado
- Inclua screenshots, gifs e outros tipos de referências que podem ajudar a explicar o problema

#### Sugerir melhorias e recursos ✨
- Use títulos claros
- Descreva a sugestão passo-a-passo
- Descreva o comportamento esperado após implementar a ideia
- Explique por que esse novo recurso ou atualização pode ser útil

#### Melhorar a documentação 📝
- Use títulos claros
- Especifique os arquivos que precisam ser documentados
- Explique suas sugestões e por que seriam melhores ou mais claras

### Pull Requests
Se você quiser contribuir com código para o projeto, procure uma issue e comece a desenvolver sua solução! Quando você estiver pronto, abra uma Pull Request e nós vamos revisar.

Algumas recomendações:

- Descreva exatamente o que você fez e porque, sendo sempre o mais claro possível
- Adicione o link com a issue correspondente ao seu Pull Request (se não houver nenhuma, por favor crie uma nova)
- Confira se você está seguindo o [Syleguide](#Styleguide)

## Styleguide 💄
O código e a estrutura devem seguir [ROS Use Patterns](http://wiki.ros.org/ROS/Patterns) e [ROS Best Practices](http://wiki.ros.org/BestPractices).

### Python
Todo código em python deve seguir os guias [ROS Python](http://wiki.ros.org/PyStyleGuide) e [PEP 8](https://www.python.org/dev/peps/pep-0008/).

### Menssagens de commit
- As menssagens devem ser escritas preferencialmente em inglês.
- Use o tempo presente ("Add feature" não "Added feature")
- Use o modo imperativo ("Move cursor to..." não "Moves cursor to...")
- Nós recomendamos fortemente iniciar a menssagem de commit com um emoji relacionado
  - 📝 `:memo:` para documentação
  - 🐛 `:bug:` para bugs
  - 🚑 `:ambulance:` para correções críticas
  - 🎨 `:art:` para melhorias na estrutura
  - ✨ `:sparkles:` para novos recursos
  
  Para mais exemplos, veja [aqui](https://gitmoji.carloscuesta.me/).

### Fluxo de trabalho
O fluxo de trabalho é baseado no [Git Flow](https://nvie.com/posts/a-successful-git-branching-model/).

### Documentação
A documentação é gerada com [Doxygen](https://www.doxygen.nl/index.html) e deve seguir o seu manual de documentação.
