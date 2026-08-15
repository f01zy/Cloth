# Preview

![preview](assets/preview.jpg)

# Building

```bash
git clone --recurse-submodules https://github.com/f01zy/Cloth && cd Cloth 
mkdir build && cd build
cmake ..
cmake --build .
```

# Controls

## Keyboard

| Key | Action                             |
|-----|------------------------------------|
| Q   | Pin the cloth's top-left point     |
| W   | Pin the cloth's top-right point    |
| A   | Pin the cloth's bottom-left point  |
| S   | Pin the cloth's bottom-right point |
| R   | Reset the cloth's position         |
| F1  | Enable wireframe mode              |

## Mouse

| Key          | Action                    |
|--------------|---------------------------|
| MOUSE LEFT   | Rotate the camera         |
| MOUSE RIGHT  | Drag the cloth            |
| MOUSE SCROLL | Zoom the camera in or out |
