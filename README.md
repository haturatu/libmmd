# libmmd

libmmd は、MikuMikuDance (MMD) の PMX モデル、VMD モーション、VPD ポーズを扱う C++20 ライブラリです。モデルの読み書き、モーション評価、モデル編集、オプションの物理シミュレーションをアプリケーションに組み込めます。

## 主な機能

- PMX モデルとメッシュの読み込み、検証、保存
- VMD モーションと VPD ポーズの読み込み
- ボーン、モーフ、IK、スキニングを含むアニメーション評価
- ハンドルとトランザクションを使った PMX モデル編集
- Bullet を使った剛体物理と、PMX 2.1 ソフトボディ用のシミュレーション

## 必要な環境

- CMake 3.25 以降
- C++20 に対応するコンパイラー
- Windows 以外では Iconv
- 剛体物理を使う場合は Bullet

Bullet は既定で検出されます。Bullet が見つからない場合も物理モジュールはビルドされますが、`MmdPhysics::available()` は `false` を返します。PMX 2.1 ソフトボディ用シミュレーションは Bullet を必要としません。

## ビルドとインストール

リポジトリのルートで次を実行します。

```sh
cmake -S . -B build
cmake --build build
cmake --install build --prefix "$PWD/install"
```

Bullet の検出や依存関係の取得は、次の CMake オプションで切り替えられます。

| オプション | 既定値 | 説明 |
| --- | --- | --- |
| `LIBMMD_ENABLE_PHYSICS` | `ON` | 物理モジュールをビルドします。`OFF` の場合、`mmd::physics` は生成されません。 |
| `LIBMMD_ENABLE_BULLET` | `ON` | システムにインストールされた Bullet を検出して使います。 |
| `LIBMMD_FETCH_DEPENDENCIES` | `OFF` | Bullet が見つからない場合にソースを取得します。これを有効にするときは `LIBMMD_INSTALL=OFF` も指定してください。 |

物理モジュールを含めずにビルドするには、次を実行します。

```sh
cmake -S . -B build-no-physics -DLIBMMD_ENABLE_PHYSICS=OFF
cmake --build build-no-physics
```

## CMake プロジェクトで使う

インストールした libmmd を使うアプリケーションの `CMakeLists.txt` に、次を追加します。

```cmake
find_package(libmmd CONFIG REQUIRED)
target_link_libraries(app PRIVATE mmd::core mmd::animation mmd::physics)
```

物理モジュールを無効にしてビルドした場合は、`mmd::physics` をリンク対象から外してください。libmmd のソースをサブディレクトリとして追加する場合も、同じターゲット名を使えます。

```cmake
add_subdirectory(external/libmmd)
target_link_libraries(app PRIVATE mmd::core mmd::animation)
```

## PMX とモーションを読み込む

次の例では、PMX モデルと VMD モーションを読み込み、30 フレーム目のモデル状態を評価します。

```cpp
#include <mmd/animation.hpp>
#include <mmd/pmx.hpp>
#include <mmd/vmd.hpp>

const auto model = mmd::pmx::load("model.pmx");
const auto motion = mmd::loadVmd("motion.vmd");

mmd::MmdAnimator animator(model);
animator.setMotion(&motion);
const auto frame = animator.evaluate(30.0F, 1.0F / 60.0F);
```

`frame` には評価後の頂点、ボーン、マテリアルなどが含まれます。描画やアプリケーション固有の処理は呼び出し側で行います。モデル、モーション、ポーズ、物理ワールドは、アニメーターが参照している間は破棄しないでください。

VPD ポーズは `mmd::loadVpd()` で読み込み、`animator.setPose(&pose)` で設定できます。VPD に含まれるボーンのローカル位置と回転が VMD の値を上書きします。`animator.setPose(nullptr)` で上書きを解除できます。

剛体物理を使う場合は `mmd::MmdPhysics` を作成し、`animator.setPhysics(&physics)` でアニメーターに設定します。Bullet が利用できるかどうかは `physics.available()` で確認できます。

編集したモデルは `mmd::pmx::save()` で保存できます。

```cpp
const auto report = mmd::pmx::save("edited.pmx", model);
```

既定の保存モードは PMX のエンコーディングとインデックス幅を保持し、必要に応じてインデックス幅を広げます。

## API リファレンス

- [PMX の読み書きとデータ型](include/mmd/pmx.hpp)
- [アニメーション評価](include/mmd/animation.hpp)
- [VMD と VPD](include/mmd/vmd.hpp)
- [物理シミュレーション](include/mmd/physics.hpp)
- [トランザクション編集](include/mmd/document.hpp)

## ライセンス

libmmd は [MIT License](LICENSE) で提供されます。第三者コードに関する情報は [THIRD_PARTY_NOTICES.md](THIRD_PARTY_NOTICES.md) を参照してください。
