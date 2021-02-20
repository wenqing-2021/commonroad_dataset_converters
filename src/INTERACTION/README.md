### INTERACTION Dataset Converter

This script converts [INTERACTION](https://interaction-dataset.com/) dataset into CommonRoad scenarios, which consists of 15016 individual planning scenarios, and 2221 cooperative planning scenarios.

- You need to request the original dataset via [this link](https://docs.google.com/forms/d/e/1FAIpQLSdX1XM14idrtHEd9HIiUMCCbiiQZlvwJTaixY8U4PfXCqJ5Zg/viewform), and put the **INTERACTION-Dataset-DR-v1_0** and **INTERACTION-Dataset-TC-v1_0** folders under folder **dataset**.

- To start conversion, simply use the following command:

  ```shell
  python converter_INTERACTION.py
  ```

- To generate GIF animations of the newly converted scenarios, simply use the following command:

  ```shell
  python create_gifs.py
  ```

  