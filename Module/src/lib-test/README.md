# How To Use lib-test
This folder is designed to treat the `lib` folder as a package, utilizing relative imports to access scripts within the `lib` folder.

To run a file inside `lib-test` to test the `lib` folder scripts, use the following commands:

1.  **Navigate to your project's root directory** (the folder containing `src`):
    ```bash
    cd /home/ubuntu/Documents/programs/CV/Module/
    ```

2.  **Run the script as a module** (without the `.py` extension, and prefixing with `src.` to indicate its package path):
    ```bash
    python3 -m src.lib-test.{script-name}
    ```
    *Replace `{script-name}` with the actual name of your Python file (e.g., `test_dummy_lib`).*


**Notes** 
Path is relative to Module folder, so if your code uses checkerboard_images folder inside data/images/checkerboard_images

then for the path use 
```bash
image = 'data/images/checkerboard_images'
```