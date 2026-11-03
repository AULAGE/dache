# media/：放照片、截图、视频、bench.csv

这个目录里的文件不会提交到仓库（见 `../.gitignore`），只在本机用来生成材料。

- 照片 / 截图：任意文件名，在 `../data.json` 的“照片”一节写文件名。大照片生成时会自动缩到 1800 像素。
- 视频：mp4，在 `../data.json` 的“视频”一节按拍摄清单编号（①③④⑤⑦⑧）写文件名，PPT 会嵌入对应视频位。
- `bench.csv`：从树莓派 `raspberrypi/grasp/logs/bench.csv` 拷过来，`data.json` 的 `bench.csv文件` 写 `bench.csv`。
