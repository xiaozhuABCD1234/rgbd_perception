# 贡献与提交规范

## 开发流程

1. 从最新 `main` 创建短生命周期分支。
2. 每个提交只完成一件内聚的事情，构建产物不得提交。
3. 提交前运行 `just check`；涉及内存或并发的修改再运行 `just sanitizer`。
4. 新功能附测试，缺陷修复先增加能够复现问题的测试。
5. 合并前保证编译、测试、格式检查和提交信息检查全部通过。

## Conventional Commits

提交标题格式：

```text
<type>(<optional scope>): <imperative summary>
```

允许的 type：

| 类型 | 用途 |
|---|---|
| `feat` | 新功能 |
| `fix` | 缺陷修复 |
| `docs` | 文档 |
| `style` | 不影响行为的格式调整 |
| `refactor` | 不增加功能、不修复缺陷的重构 |
| `perf` | 性能优化 |
| `test` | 测试新增或调整 |
| `build` | 构建系统或依赖 |
| `ci` | 持续集成 |
| `chore` | 其他维护工作 |
| `revert` | 回退提交 |

推荐 scope：`camera`、`core`、`perception`、`replay`、`app`、`build`、`ci`、`docs`、`test`。

规则：

- 标题不超过 72 个字符，使用祈使句，说明“做了什么”。
- 标题与正文之间保留一个空行。
- 正文解释背景和原因，而不是重复代码 diff；普通正文行不超过 100 个字符。
- 不兼容变更必须在 type/scope 后添加 `!`，并在 footer 中写 `BREAKING CHANGE:`。
- 关联问题使用 `Refs: #123`，关闭问题使用 `Closes: #123`。

示例：

```text
feat(camera): add Gemini 305 capture adapter

Convert SDK-owned buffers to self-describing frames at the adapter boundary.
Device reconnect handling is intentionally deferred to the lifecycle layer.

Refs: #12
```

## 本地钩子

运行一次：

```bash
just hooks
```

该命令设置仓库级 `core.hooksPath=.githooks` 和提交模板，不修改全局 Git 配置。`pre-commit` 检查暂存内容的空白错误，`commit-msg` 检查提交格式。

不要常规使用 `git commit --no-verify`；确需绕过时应说明原因，并由 CI 做最终检查。
