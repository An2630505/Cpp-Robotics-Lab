# Git Commit 规范指南

## 提交格式

```
<type>[module]: <subject>

<body> (optional)

<footer> (optional)
```

## 提交类型 (Type)

| 类型 | 说明 | 示例 |
|------|------|------|
| `feat` | 新功能 | `feat[PID]: add anti-windup mechanism` |
| `fix` | Bug 修复 | `fix[KF]: correct matrix dimension` |
| `docs` | 文档变更 | `docs[Main]: update README` |
| `style` | 代码格式(不影响功能) | `style[Plant]: format code` |
| `refactor` | 代码重构 | `refactor[LQR]: simplify initialization` |
| `perf` | 性能优化 | `perf[KF]: optimize matrix operations` |
| `test` | 测试相关 | `test[PID]: add unit tests` |
| `chore` | 构建/工具变更 | `chore[Build]: update Makefile` |

## 模块名称 (Module)

常用模块:
- `PID` - PID 控制器
- `LQR` - LQR 控制器
- `KF` - 卡尔曼滤波器
- `Plant` - 被控对象模型
- `Main` - 主程序
- `Build` - 构建系统
- `Docs` - 文档

## Subject 规则

- 使用祈使句、现在时: "add" 而非 "added" 或 "adds"
- 首字母小写
- 结尾不加句号
- 简洁明了,不超过 50 个字符

## Body 规则 (可选)

- 与 subject 之间空一行
- 每行不超过 72 个字符
- 解释 **what** 和 **why**,而不是 **how**

## 完整示例

### 简单提交
```bash
git commit -m "feat[PID]: implement incremental PID controller"
```

### 详细提交
```bash
git commit -m "feat[PID]: add anti-windup mechanism

Add saturation limits to prevent integral windup in PID controller.
This improves stability when the system reaches physical constraints.

Closes #123"
```

## 配置说明

本项目已配置 Git commit 模板,执行 `git commit` 时会自动打开模板文件作为参考。

模板文件位置: `.gitmessage`

## 常用工作流

```bash
# 1. 添加修改
git add .

# 2. 提交(会打开模板编辑器)
git commit

# 或者直接使用命令行
git commit -m "feat[module]: description"

# 3. 推送到远程
git push
```

## 注意事项

- ✅ 使用方括号标注模块: `feat[PID]`
- ✅ 保持提交信息简洁清晰
- ✅ 一次提交只做一件事
- ❌ 避免模糊的描述如 "update code"
- ❌ 不要在 subject 中使用过去时态
