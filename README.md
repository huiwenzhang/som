# som
C++ implementation of SOM(self-organizing map) for solving tsp problem with 3D input.


# Build & test
```
mkdir build
cd build
cmake ..
make
```

To run the code, you need to provide a `*.tsp` file which represents the problem you want to solve. To quickly test the code, you can use the demo file located under the directory `media/input.tsp`. Example run
```
mkdir data
./example ../media/input.tsp
```

The intermediate results are stored in directory named `data` which we just created under the `build` directory. By using the files under `data` folder, you can visualize the results with python plot tools.

# Results
The above demo shows a tsp problem with 88 locations, results shown below
![](./media/tsp.gif)


# References
- https://github.com/diego-vicente/som-tsp

The updates made in this project:
- adapte the code to 3D data
- provide a simple way to make the code useable for close-loop tsp problem
- to make the resulted path more smooth, the direciton change cost is introduced, but it seems to cause undesired convergence problem

