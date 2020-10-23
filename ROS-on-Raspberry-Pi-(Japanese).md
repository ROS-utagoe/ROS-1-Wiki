# ROS とは
Robot Operating System(ROS) とは， [ROS.org](https://ros.org) が開発する，ロボットソフトウェアのフレームワークです．
詳しい説明は [ROS wiki](https://wiki.ros.org) で確認できます．

具体例として，以下のセンサーから取得した値をもとに**Controller**でモーターの回転速度を決め，モーターに出力を与えるプログラムを考えます．

![](https://i.imgur.com/XC7g8Np.png)

普通のプログラムでこれを実現しようとすると，センサから値を読み取るプログラム(**Sensor A, B**)から得られた値を何らかの方法で**Controller**に渡します．
**Controller**は得られたセンサ値からモーターの制御量を計算し，**Motor**に制御量を渡し，**Motor**でモーターを制御する，という流れです．
ここで問題になるのが，値を渡す方法です．これを統一しないと，**Controller**と別のプログラム(**Controller 2**)を入れ替えようとしたとき，値を渡す経路をいちいち書き換えなくてはなりません．

そこを解決してくれるのがROSです．

## ROSのモデル図

上のプログラムのうち，センサから**Controller**までをROSで実装すると，このようになります．

![](https://i.imgur.com/VEzev8y.png)

用語を説明します．

Node
: ROSでは機能を持つプログラムをNodeと言います．

Message
: データはメッセージという形で送受信します．
メッセージにはint32やfloat32，stringなど複数のデータ型

Topic
: メッセージを受け取り，内容を知ることができる掲示板のようなものです．
このTopicに対してメッセージを送ることを***Publish***といい，このTopicの内容を受信することを***Subscribe***といいます．
Topicでやり取りされるメッセージの型は指定します．

こうすることで，各ノードは，相手を気にせずにメッセージをPublish/Subscribeできます．

**Controller**からモーターを動かすまでの流れは次のようになります．

![](https://i.imgur.com/8IJbf90.png)

ノード**Controller**はトピック`/sensors`をSubscribeし，トピック`/motor`に対しPublishする，という構造になっています．
異なるノード**Controller2**と入れ替えたい場合，同じトピックに対しPublish/Subscribeするノードを作成し，立ち上げるノードを**Controller**から**Controller2**に変更するだけで実現できます．

# Install
[こちら](https://qiita.com/Ninagawa_Izumi/items/063d9d4910a19e9fcdec)のサイトを参考にインストールしました．
最後に`.bashrc`などに`source /opt/ros/melodic/setup.bash`などと書いておくと便利です．

# 準備
ROS wiki の [Tutorials](http://wiki.ros.org/ROS/Tutorials) に従うと，ROSのの基本を理解できます．
ここでは，独自に作成する場合の注意点を書きます．
## ワークスペースの作成
[Installing and Configuring Your ROS Environment](http://wiki.ros.org/ROS/Tutorials/InstallingandConfiguringROSEnvironment)
こちらを参考にします．

ワークスペースを作ります．ホームディレクトリ直下に`~/catkin_ws/src`を作ります．
> ワークスペースの名前や場所は何でもいいですが，ホームディレクトリ直下の`~/catkin_ws`がよく使われています．
> これは私の環境ではダメだったという話ですが，ホームディレクトリ直下以外の場所にワークスペースを作ると，後に説明する`roscore`が失敗してしまいました．(原因は解明できていません)ホームディレクトリ直下で作り直したところ問題なく起動したので，異なる場所でワークスペースを作った後に問題があれば，ホームディレクトリ直下にワークスペースを作り直してみるといいと思います．


`catkin_ws`で`catkin_make`を実行します．
```
$ mkdir -p ~/catkin_ws/src
$ cd ~/catkin_ws
$ catkin_make
```
`catkin_make`により，様々な必要なファイルが生成されます．
ワークスペースの作成が終わったら，ディレクトリをROSのワークスペースであると認識させるために以下を実行します．
```
$ source ~/catkin_ws/devel/setup.bash
```
これは++新しく開くターミナル全て++で実行する必要があります．
ただ，毎回やるのは面倒なので，こちらも`~/.bashrc`に書いておくといいと思います．

## パッケージ
既存のパッケージを使ってまずは試してみたい場合は，ROS wiki の [Tutorials](http://wiki.ros.org/ROS/Tutorials) に従ってください．
オリジナルのパッケージを作成する場合は，[Creating an ROS Package](http://wiki.ros.org/ROS/Tutorials/CreatingPackage)に従います．

### パッケージの作成
パッケージは`~/catkin_ws/src`に作成します．

    $ cd ~/catkin_ws/src
    $ catkin_create_pkg <パッケージ名> std_msgs rospy roscpp
    
`<パッケージ名>`の後には依存パッケージを書きます．上で示した3つは必要最低限なので，入れておきます．





