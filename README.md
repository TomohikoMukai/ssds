# SSDS for Maya 2018
本コードは、[Sampling-based Rig Conversion into Non-rigid Helper Bones](https://mukai-lab.org/publications/i3d2018/ "SSDS paper")で提案している"Skinning Decomposition with Similarity Transformations (SSDS)"のMaya2018用プラグイン実装のサンプルです。任意のメッシュアニメーションを、ジョイントアニメーション（平行移動＋回転＋スケール）に変換します。[SSDR4Maya](https://github.com/TomohikoMukai/ssdr4maya)の上位版です。また、[Locality-Aware Skinning Decomposition Using Model-Dependent Mesh Clustering](https://sites.google.com/view/fumiyanarita/project/la_ssdr_mdmc)で提案しているアルゴリズムの一部も実装しています。

- Tomohiko Mukai, Sampling-based Rig Conversion into Non-rigid Helper Bones, Proceedings of the ACM on Computer Graphics and Interactive Techniques, 1(1), Article 13. (ACM SIGGRAPH Symposium on
Interactive 3D Graphics and Games 2018).
- Fumiya Narita and Tomohiko Mukai, Locality-Aware Skinning Decomposition Using Model-Dependent Mesh Clustering, Computer Graphics International 2020.

## インストール方法
modules/ssds2018フォルダ一式を、Maya2018のモジュールフォルダに置きます（[参考](https://help.autodesk.com/view/MAYAUL/2018/JPN/?guid=__files_GUID_130A3F57_2A5D_4E56_B066_6B86F68EEA22_htm)）

![インストール](https://github.com/TomohikoMukai/ssds/blob/image/install.png)

また、numpyをMAYA_SCRIPTS_PATHが通ったフォルダにインストールしておく必要もあります。Maya2018で動作するバイナリは[このページ](https://mukai-lab.org/library/mayanumpy/)の最下部に公開しています。

![numpy](https://github.com/TomohikoMukai/ssds/blob/image/numpy.png)

## 使い方
plugin manager から「mlSSDS.py」をロードすると、メニューから「MukaiLab→SSDS」を起動できるようになります。その後、ジョイントアニメーションに変換したいメッシュを選択した上で、選択して実行します。
  * 使用するジョイントの数 #Joints
     * 最小数と最大数を指定します。最小数、最大数を用いた推定結果に加え、(最小数+最大数)/2 つのジョイントを用いた計3パターンが生成されます
  * 最大インフルーエンス数 Influences
     * 頂点あたりの最大インフルーエンス数
  * 計算反復回数 #Interations
     * 回数を増やすほど正確な結果に近づくことが期待できますが、あまり大きくしすぎても効果は薄いです。
  * ウェイト分布の滑らかさに関する設定 Locality
     * None/1-ring/2-ring：各頂点に影響するジョイントを絞り込みます。Noneは絞り込みなし、1-ringは最近傍のジョイントのみ、2-ringはその周辺のジョイントまで含めたウェイト最適化を行います。（詳しいアルゴリズムは [CGI2020発表論文](https://sites.google.com/view/fumiyanarita/project/la_ssdr_mdmc) を参照してください）
     * 0～10の数字は、各頂点とその周辺頂点とのウェイトのスムーズさを指定します。0では頂点ごとに独立にウェイトを推定し、値が大きくなるにつれ周辺頂点とのウェイトの類似性を重要視するような最適化を行います。
  * 初期ジョイント配置決定法 Sampling
     * チェックボックスを入れるとメッシュ上になるべく均一にジョイントを初期配置します。チェックを外すと、最大誤差を示す頂点から順番にジョイントを反復的に配置します。
  * トランスフォーム種別 TransformType
     * T: 平行移動のみ
     * T+R: 平行移動＋回転
     * T+R+S: 平行移動＋回転＋スケール


![実行結果](https://github.com/TomohikoMukai/ssds/blob/image/result.png)
2063頂点のモデルに対して、最小20ジョイント～最大40ジョイント、平行移動のみ/平行移動+回転+スケールの計算を施した結果の一例を示します。

## 動作環境
Windows 10 + Maya 2018 (無印)の環境でのみ開発・動作確認しています。その他の環境については未サポートです。

## ソースコードについて
ネイティブコードのビルドには次のライブラリが必要です。
* Eigen 3.2.10: http://eigen.tuxfamily.org/
* OSQP 0.6.2: https://osqp.org/
* numpy: https://numpy.org/


## 更新履歴
- [2020.9.6] DLLのソースコード公開、バッチ処理の追加、改良アルゴリズムの搭載、その他の細かいバグ修正
- [2019.1.23] ウェイト最適化法の安定化と細かいバグ修正
- [2019.1.21] ウェイト集中化パラメータの追加、計算の安定化など
- [2018.12.24] 並列計算まわりのバグ修正
- [2018.11.13] 細かいバグ修正と高速化
