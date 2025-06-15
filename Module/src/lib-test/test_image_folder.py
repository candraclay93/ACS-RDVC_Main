import os

path = "data/images/checkerboard_images/lt1_pantry"

if os.path.isdir(path):
    print(f"{path} is a dir")
    # try:
    #     for f in os.path.isdir(path):
    #         print(f)
    # except Exception as e:
    #     print(e)
else:
    print(f"{path} not a dir")