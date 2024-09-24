This folder contains code to generate a map (.csv) file that given a set of coordinates (x,y) and a timestamp t gives you the acceleration data for that point at that time.

Move to code directory:
```bash
cd python_generate_map/
```

Create environment:
```bash
conda env create -f environment.yml
```

Activate environment:
```bash
conda activate WeBots_TinyMl
```

Run code:
```bash
python3 generate_map_vibration_data.py
```
