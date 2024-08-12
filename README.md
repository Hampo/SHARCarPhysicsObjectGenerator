# SHARCarPhysicsObjectGenerator
A CLI application to generate Physics Objects for vehicles in Simpsons: Hit & Run.

# How to use
```
Usage: SHARCarPhysicsObjectGenerator [options] <input_path> [output_path]

Options:
  -f  | --force         Force overwrite the output file.
  -nh | --no_history    Don't add history chunk.
  -nr | --no_remove     Don't remove existing PhysicsObject chunks, they will renamed instead.

Arguments:
  <input_path>   The input car's P3D file.
  [output_path]  The output P3D file. If omitted, it will attempt to overwrite "input_path".

Example:
  SHARCarPhysicsObjectGenerator C:\input\car.p3d C:\output\car.p3d
  SHARCarPhysicsObjectGenerator --force --no_history C:\input\car.p3d
```