# 团队开发工作流与 Git 使用规范（ROS2 课程项目）

> 本文作为 **ros2-openarm-project** 的统一开发规范，供所有组员参考与遵守。目的是：
>
> * 避免分支混乱、代码冲突
> * 保证 ROS2 工程结构清晰
> * 提高多人协作效率

---

## 一、总体原则（请先读）

1. **禁止在 `main` 分支直接开发**
2. **所有人从 `dev` 分支拉代码**
3. **每个成员只在自己的功能分支写代码**
4. **通过 Pull Request 合并到 `dev`**
5. **ROS2 package 一律作为普通目录管理（禁止 submodule）**

---

## 二、分支说明

### 1️⃣ main 分支（最终成果）

* 用途：

  * 课程验收
  * 最终展示
* 规则：

  * ❌ 禁止直接 push
  * ✅ 仅在项目阶段性完成后，由负责人从 `dev` 合并

---

### 2️⃣ dev 分支（日常集成分支）

* 用途：

  * 汇总所有成员的阶段性成果
* 规则：

  * ❌ 不直接写代码
  * ✅ 只通过 Pull Request 合并

---

### 3️⃣ 功能分支（个人开发分支）

* 命名规范（示例）：

```text
feat/A-simulation
feat/B-motion-control
feat/C-perception
fix/B-ik-bug
```

* 原则：

  * 一个分支 = 一个功能 / 模块
  * 功能完成后即合并并删除分支

---

## 三、标准开发流程（每位成员必须遵守）

### Step 1：首次获取代码

```bash
git clone git@github.com:robotcourse1/ros2-openarm-project.git
cd ros2-openarm-project
git checkout dev
git pull
```

---

### Step 2：创建自己的功能分支

```bash
git checkout -b feat/你的名字-模块名
```

示例：

```bash
git checkout -b feat/B-motion-control
```

---

### Step 3：编写代码（本地）

* 代码位置要求：

  * 每个 ROS2 功能必须放在对应目录
  * 不随意新建顶层目录

* 开发过程中常用命令：

```bash
git status
git diff
```

---

### Step 4：提交代码

```bash
git add .
git commit -m "feat(B): add motion control node"
```

#### 提交信息规范（建议）

```text
feat: 新功能
fix: 修复 bug
chore: 构建 / 配置 / 结构调整
docs: 文档更新
```

---

### Step 5：保持分支最新（非常重要）

在 push 前，必须先同步 dev：

```bash
git checkout dev
git pull --rebase
git checkout feat/B-motion-control
git rebase dev
```

> 目的：减少合并冲突

---

### Step 6：推送并提 Pull Request

```bash
git push origin feat/B-motion-control
```

* 到 GitHub：

  * 提 PR → `dev`
  * 简要说明做了什么

---

## 四、常见错误与禁止行为

### ❌ 1. 直接在 dev / main 写代码

风险：

* 容易覆盖他人代码
* 历史不可追踪

---

### ❌ 2. 提交带 `.git` 的目录

风险：

* 产生 submodule
* GitHub 无法浏览文件

检查方法：

```bash
ls -a 包名
```

如发现 `.git`，必须删除：

```bash
rm -rf 包名/.git
```

---

### ❌ 3. 不看 git status 就操作

这是造成 90% Git 问题的根源。

---

## 五、跨系统开发说明（Windows / Ubuntu）

* Ubuntu：

  * 用于 ROS2 运行与验证
* Windows：

  * 可用于代码编辑、文档编写

注意事项：

* 出现 `LF / CRLF` 警告属于正常现象
* 不影响功能，无需惊慌

---

## 六、冲突处理原则

* 小冲突：

  * 自行解决后继续 rebase

* 大冲突：

  * 立即在群内说明
  * 不要强行 push

---

## 七、最终检查清单（提交前必看）

在你提交 PR 前，请确认：

* [ ] 在自己的功能分支
* [ ] `git status` 显示干净
* [ ] 已 rebase 最新 dev
* [ ] 不包含无关文件
* [ ] 能通过基本 ROS2 build / launch

---


**维护人：项目组 Git 负责人**

**版本：v1.0**
