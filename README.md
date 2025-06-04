## Install packages
1. Install `pyenv` from [here](https://github.com/pyenv/pyenv?tab=readme-ov-file#a-getting-pyenv)
```
pyenv install 3.12.8
```
2. Install `pyenv virtualenv` from [here](https://github.com/pyenv/pyenv-virtualenv)

```
pyenv virtualenv 3.12.8 cobotta
pyenv local cobotta
```
3. Install packages
```
pip install -r requirements.txt
```

## Run pybullet simulation for Cobotta
```python
python pyb_cobotta.py
#python pyb_cobotta_random.py #for random simulation
```
