if [ -d build ]; then
    echo "build does exist."
else
    echo "create build file."
    mkdir build
fi

cd build
cmake ..
make

