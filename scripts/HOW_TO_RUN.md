# 如何运行 Claude 给的脚本（速查表）

## 🎯 起手式（每次都一样）

```bash
# ①  准备接收粘贴：
cat > /tmp/s.sh

# ②  从浏览器 Ctrl+C 的内容 → 右键粘贴到终端
# ③  粘贴完按 Enter → 按 Ctrl+D 结束

# ④  运行并保存输出：
bash /tmp/s.sh 2>&1 | tee /tmp/out.txt
```

## 📋 完整流程

1. 浏览器打开脚本页面，例如：
   `https://github.com/phyllis-TANG/gnss_ws/blob/claude/review-gnss-spp-JmtgD/scripts/02_inspect_ublox_driver.sh`

2. 点右上角 **"Raw"** 按钮（纯文本视图）

3. `Ctrl+A` 全选 → `Ctrl+C` 复制

4. 终端执行：`cat > /tmp/s.sh`

5. 右键粘贴 → `Enter` → `Ctrl+D`

6. 执行：`bash /tmp/s.sh 2>&1 | tee /tmp/out.txt`

7. 把 `/tmp/out.txt` 上传到 GitHub 的 `scripts/` 目录，告诉 Claude

## 💡 记忆口诀

**Cat 接住，Bash 跑，Tee 存一份**

| 命令 | 作用 |
|------|------|
| `cat > /tmp/s.sh` | 接住粘贴的脚本内容 |
| `bash /tmp/s.sh` | 运行脚本 |
| `tee /tmp/out.txt` | 同时屏幕显示 + 写入文件 |

## ⚠️ 常见问题

- **粘贴时有些内容丢失** → 用鼠标右键粘贴，不要用 Ctrl+V（某些终端会吃掉特殊字符）
- **Ctrl+D 没反应** → 确保光标在新的一行，而不是脚本内容的最后一行末尾
- **想查看已下载的脚本** → `cat /tmp/s.sh`
