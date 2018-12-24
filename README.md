# SSDS for Maya 2018
本コードは、[Sampling-based Rig Conversion into Non-rigid Helper Bones](https://mukai-lab.org/publications/i3d2018/ "SSDS paper")で提案している"Skinning Decomposition with Similarity Transformations (SSDS)"のMaya2018用プラグイン実装のサンプルです。任意のメッシュアニメーションを、ボーンアニメーション（平行移動＋回転＋スケール）に変換します。[SSDR4Maya](https://github.com/TomohikoMukai/ssdr4maya)の上位版です。

- Tomohiko Mukai, Sampling-based Rig Conversion into Non-rigid Helper Bones, Proceedings of the ACM on Computer Graphics and Interactive Techniques, 1(1), Article 13. (ACM SIGGRAPH Symposium on
Interactive 3D Graphics and Games 2018).

## インストール方法
modules/ssds2018フォルダ一式を、Maya2018のモジュールフォルダに置きます（[参考](https://help.autodesk.com/view/MAYAUL/2018/JPN/?guid=__files_GUID_130A3F57_2A5D_4E56_B066_6B86F68EEA22_htm)）

![インストール](https://github.com/TomohikoMukai/ssds/blob/image/install.png)

また、numpyをMAYA_SCRIPTS_PATHが通ったフォルダにインストールしておく必要もあります。Maya2018で動作するバイナリは[このページ](https://mukai-lab.org/library/mayanumpy/)の最下部に公開しています。

![numpy](https://github.com/TomohikoMukai/ssds/blob/image/numpy.png)

## 使い方
plugin manager から「mlSSDS.py」をロードすると、メニューから「MukaiLab→SSDS」を起動できるようになります。その後、ボーンアニメーションに変換したいメッシュを選択した上で、使用するボーン、トランスフォーム種別（平行移動のみ、平行移動＋回転、平行移動＋回転＋スケール）を選択して実行します。

![使い方](https://github.com/TomohikoMukai/ssds/blob/image/execute.png)

1200頂点に対して30個のボーンを用いた結果の一例を示します。

![実行結果](https://github.com/TomohikoMukai/ssds/blob/image/result.png)

## 動作環境
Windows 10 + Maya 2018 (無印)の環境でのみ開発・動作確認しています。その他の環境については未サポートです。

## ソースコードについて
研究用途・商用利用でご興味のある方は contact@mukai-lab.org までご連絡ください。

## 更新履歴
- [2018.12.24] 並列計算まわりのバグ修正
- [2018.11.13] 細かいバグ修正と高速化
