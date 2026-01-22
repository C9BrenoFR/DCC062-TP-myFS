# Lembre de rodar chmod +x compile.sh
# Esse comando compila, executa e apaga o executavel em seguida.

# Compila
gcc -o main main.c disk.c inode.c util.c vfs.c myfs.c -Wall

# Executa (Com parametro)
./main $1

# Removendo executavel
rm main 