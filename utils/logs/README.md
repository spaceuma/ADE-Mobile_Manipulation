Simple utility to extract log information and represent it.
How to use it:

./logExtraction.sh /path/to/the/log/directory

The script will extract the MM logs and create a directory called exctractedLog/ where all the information can be easily accessed by the python scripts. Then, for example:

python3 MM_navigationMapsViewer.py

Will represent the resulting cost map, path and traversability map. Some other scripts are also available.
