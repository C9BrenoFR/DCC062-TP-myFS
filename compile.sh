# Lembre de rodar chmod +x compile.sh
# Esse comando compila, executa e apaga o executavel em seguida.

# Compila
g++ main.cpp -o main

# Executa (Com parametro)
./main $1

# Removendo executavel
rm main 