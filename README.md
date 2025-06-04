编译：
1.cmake -G "MinGW Makefiles" -DCMAKE_C_COMPILER=D:/openCV/mingw64-8.1.0/bin/gcc.exe -DCMAKE_CXX_COMPILER=D:/openCV/mingw64-8.1.0/bin/g++.exe -DOpenCV_DIR=D:/openCV/OpenCV-MinGW-Build-4.5.2/x64/mingw/lib ..
2.mingw32-make

git规范：下面是对代码修改后执行流程的分步说明：

### 1. 在本地其它分支上commit
```bash
# 确保在你的功能分支上
git status  # 查看状态
git add .   # 添加所有修改
git commit -m "完成功能开发"
```

### 2. 切换到develop分支并拉取最新代码
```bash
git checkout develop          # 切换到develop分支
git pull origin develop       # 拉取远程develop最新代码
```

### 3. cherry-pick特定提交
```bash
git cherry-pick <commit_id>   # 将指定提交应用到当前分支
# 可能需要解决冲突
git add .                     # 冲突解决后添加文件
git cherry-pick --continue    # 继续cherry-pick过程
```

### 4. 编译项目
```bash
# 根据项目类型执行编译命令
mvn clean package  # 示例：Maven项目
npm install        # 示例：前端项目
npm run build
```

### 5. 查看远程仓库是否有新提交
```bash
git fetch origin          # 获取远程仓库最新信息但不合并
git log develop..origin/develop  # 查看远程比本地多的提交
```

### 6. 没有新提交时推送代码
```bash
git push -u origin develop  # 首次推送使用-u参数关联上游分支
# 后续推送可直接使用：git push
```

### 7. 代码同步（假设同步到另一个仓库）
```bash
cd /path/to/your/project    # 进入项目目录
git remote add target_repo <仓库地址>  # 添加远程仓库别名
git fetch target_repo       # 获取目标仓库所有分支信息
```

### 8. 同步特定分支示例
```bash
git checkout -b new_branch target_repo/develop  # 创建并切换到新分支
git push origin new_branch:develop  # 推送到当前仓库的develop分支
```

### 关键注意点：
1. cherry-pick前确保develop是最新状态
2. 编译失败时需解决代码问题再提交
3. 推送前再次确认远程状态避免冲突
4. 多远程仓库操作时注意分支名称映射
5. 冲突解决后使用`git cherry-pick --continue`继续

建议在操作前备份重要数据，复杂操作可使用`git reflog`记录引用变更历史，以便回退操作。
git log --oneline -n 3  # 查看最近3条提交记录
git branch -vv  # 查看本地分支与远程分支的关联关系
# 如果你想将本地develop分支推送到target_repo的main分支（需要根据目标仓库的分支名调整）
git push target_repo develop:main
# 查看所有已添加的远程仓库
git remote -v

# 查看某个远程仓库的详细信息
git remote show target_repo

# 删除一个远程仓库
git remote remove target_repo