/*
*  myfs.c - Implementacao do sistema de arquivos MyFS
*
*  Autores: SUPER_PROGRAMADORES_C
*  Projeto: Trabalho Pratico II - Sistemas Operacionais
*  Organizacao: Universidade Federal de Juiz de Fora
*  Departamento: Dep. Ciencia da Computacao
*
*/

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "myfs.h"
#include "vfs.h"
#include "inode.h"
#include "util.h"

//=============================================================================
// Definicoes e constantes
//=============================================================================

#define MYFS_SUPERBLOCK_SECTOR 0    // Setor do superbloco
#define MYFS_BITMAP_SECTOR 1        // Setor do bitmap de blocos
#define MYFS_ROOT_INODE 1           // Inode do diretorio raiz

// Identificador do sistema de arquivos
#define MYFS_ID 0x4D

// Offsets no superbloco (em bytes, cada campo ocupa sizeof(unsigned int))
#define SB_MAGIC 0              // Magic number para identificar o FS
#define SB_BLOCKSIZE 1          // Tamanho do bloco em bytes
#define SB_NUMBLOCKS 2          // Numero total de blocos de dados
#define SB_FIRSTDATABLOCK 3     // Primeiro setor da area de dados
#define SB_NUMINODES 4          // Numero maximo de inodes

//=============================================================================
// Estruturas de dados globais
//=============================================================================

// Estrutura do superbloco em memoria
typedef struct {
    unsigned int magic;           // Magic number (0x4D595346 = "MYSF")
    unsigned int blockSize;       // Tamanho do bloco em setores
    unsigned int numBlocks;       // Numero total de blocos de dados
    unsigned int firstDataBlock;  // Primeiro setor da area de dados
    unsigned int numInodes;       // Numero maximo de inodes
} SuperBlock;

// Estrutura para descritor de arquivo aberto
typedef struct {
    int isOpen;                   // 1 se aberto, 0 se fechado
    unsigned int inodeNumber;     // Numero do inode associado
    unsigned int cursor;          // Posicao atual do cursor no arquivo
    Disk *disk;                   // Disco associado
} FileDescriptor;

// Variaveis globais
static SuperBlock superblock;              // Superbloco em memoria
static Disk *mountedDisk = NULL;           // Disco montado
static unsigned char bitmap[DISK_SECTORDATASIZE]; // Bitmap de blocos em memoria
static FileDescriptor fdTable[MAX_FDS];    // Tabela de descritores de arquivo

//=============================================================================
// Funcoes auxiliares internas
//=============================================================================

// Carrega o superbloco do disco para a memoria
static int loadSuperblock(Disk *d) {
    unsigned char sector[DISK_SECTORDATASIZE];
    if (diskReadSector(d, MYFS_SUPERBLOCK_SECTOR, sector) < 0) return -1;
    
    char2ul(&sector[SB_MAGIC * sizeof(unsigned int)], &superblock.magic);
    char2ul(&sector[SB_BLOCKSIZE * sizeof(unsigned int)], &superblock.blockSize);
    char2ul(&sector[SB_NUMBLOCKS * sizeof(unsigned int)], &superblock.numBlocks);
    char2ul(&sector[SB_FIRSTDATABLOCK * sizeof(unsigned int)], &superblock.firstDataBlock);
    char2ul(&sector[SB_NUMINODES * sizeof(unsigned int)], &superblock.numInodes);
    
    return 0;
}

// Salva o superbloco da memoria para o disco
static int saveSuperblock(Disk *d) {
    unsigned char sector[DISK_SECTORDATASIZE];
    memset(sector, 0, DISK_SECTORDATASIZE);
    
    ul2char(superblock.magic, &sector[SB_MAGIC * sizeof(unsigned int)]);
    ul2char(superblock.blockSize, &sector[SB_BLOCKSIZE * sizeof(unsigned int)]);
    ul2char(superblock.numBlocks, &sector[SB_NUMBLOCKS * sizeof(unsigned int)]);
    ul2char(superblock.firstDataBlock, &sector[SB_FIRSTDATABLOCK * sizeof(unsigned int)]);
    ul2char(superblock.numInodes, &sector[SB_NUMINODES * sizeof(unsigned int)]);
    
    return diskWriteSector(d, MYFS_SUPERBLOCK_SECTOR, sector);
}

// Carrega o bitmap do disco para a memoria
static int loadBitmap(Disk *d) {
    return diskReadSector(d, MYFS_BITMAP_SECTOR, bitmap);
}

// Salva o bitmap da memoria para o disco
static int saveBitmap(Disk *d) {
    return diskWriteSector(d, MYFS_BITMAP_SECTOR, bitmap);
}

// Verifica se um bloco esta livre no bitmap (0 = livre, 1 = ocupado)
static int isBlockFree(unsigned int blockNum) {
    unsigned int byteIndex = blockNum / 8;
    unsigned int bitIndex = blockNum % 8;
    return !(bitmap[byteIndex] & (1 << bitIndex));
}

// Marca um bloco como ocupado no bitmap
static void setBlockUsed(unsigned int blockNum) {
    unsigned int byteIndex = blockNum / 8;
    unsigned int bitIndex = blockNum % 8;
    bitmap[byteIndex] |= (1 << bitIndex);
}

// Marca um bloco como livre no bitmap
static void setBlockFree(unsigned int blockNum) {
    unsigned int byteIndex = blockNum / 8;
    unsigned int bitIndex = blockNum % 8;
    bitmap[byteIndex] &= ~(1 << bitIndex);
}

// Encontra um bloco livre e retorna seu numero + 1, ou 0 se nao encontrar
// Retorna blockNum + 1 para diferenciar "bloco 0" de "nenhum bloco"
static unsigned int findFreeBlock(void) {
    for (unsigned int i = 0; i < superblock.numBlocks; i++) {
        if (isBlockFree(i)) {
            return i + 1;  // Retorna indice + 1
        }
    }
    return 0; // Nenhum bloco livre encontrado
}

// Converte numero do bloco para endereco de setor no disco
static unsigned int blockToSector(unsigned int blockNum) {
    return superblock.firstDataBlock + (blockNum * superblock.blockSize);
}

// Encontra um descritor de arquivo livre
// Retorna valores de 1 a MAX_FDS (main.c espera fd > 0)
static int findFreeFD(void) {
    for (int i = 0; i < MAX_FDS; i++) {
        if (!fdTable[i].isOpen) {
            return i + 1;  // Retorna indice + 1
        }
    }
    return -1;
}

// Inicializa a tabela de descritores de arquivo
static void initFDTable(void) {
    for (int i = 0; i < MAX_FDS; i++) {
        fdTable[i].isOpen = 0;
        fdTable[i].inodeNumber = 0;
        fdTable[i].cursor = 0;
        fdTable[i].disk = NULL;
    }
}

//=============================================================================
// Implementacao das funcoes do sistema de arquivos
//=============================================================================

//Funcao para verificacao se o sistema de arquivos está ocioso, ou seja,
//se nao ha quisquer descritores de arquivos em uso atualmente. Retorna
//um positivo se ocioso ou, caso contrario, 0.
int myFSIsIdle (Disk *d) {
    for (int i = 0; i < MAX_FDS; i++) {
        if (fdTable[i].isOpen && fdTable[i].disk == d) {
            return 0; // Existe descritor aberto para este disco
        }
    }
    return 1; // Sistema de arquivos ocioso
}

//Funcao para formatacao de um disco com o novo sistema de arquivos
//com tamanho de blocos igual a blockSize. Retorna o numero total de
//blocos disponiveis no disco, se formatado com sucesso. Caso contrario,
//retorna -1.
//
// DETALHES DA IMPLEMENTACAO:
// 1. Valida parametros e calcula layout do disco
// 2. Cria superbloco com metadados do sistema de arquivos
// 3. Inicializa bitmap de alocacao de blocos (todos livres)
// 4. Cria estrutura de inodes (importante: inicializa TODOS os inodes)
// 5. Configura inode do diretorio raiz
//
// LAYOUT DO DISCO:
// Setor 0: Superbloco (metadados do FS)
// Setor 1: Bitmap de blocos (controle de alocacao)
// Setores 2+: Area de inodes (estruturas de metadados de arquivos)
// Setores restantes: Area de dados (conteudo dos arquivos)
int myFSFormat (Disk *d, unsigned int blockSize) {
    if (!d || blockSize == 0) return -1;
    
    unsigned long numSectors = diskGetNumSectors(d);
    if (numSectors < 4) return -1; // Precisa de pelo menos: superbloco, bitmap, inodes, dados
    
    // Calcula o tamanho do bloco em setores
    // Um bloco e' a unidade de alocacao (pode conter multiplos setores)
    unsigned int blockSizeInSectors = blockSize / DISK_SECTORDATASIZE;
    if (blockSizeInSectors == 0) blockSizeInSectors = 1;
    
    // Calcula area de inodes (comeca no setor 2, apos superbloco e bitmap)
    unsigned int inodeAreaStart = inodeAreaBeginSector(); // Retorna 2
    unsigned int inodesPerSector = inodeNumInodesPerSector(); // Quantos inodes cabem em 1 setor
    
    // Define numero maximo de inodes
    // Reserva setores suficientes para ~64 inodes (ajustavel conforme necessidade)
    unsigned int inodeSectors = (64 + inodesPerSector - 1) / inodesPerSector;
    unsigned int numInodes = inodeSectors * inodesPerSector;
    
    // Primeiro setor de dados vem logo apos a area de inodes
    unsigned int firstDataSector = inodeAreaStart + inodeSectors;
    
    // Calcula numero de blocos de dados disponiveis
    // Cada bloco ocupa blockSizeInSectors setores consecutivos
    unsigned int dataSectors = numSectors - firstDataSector;
    unsigned int numBlocks = dataSectors / blockSizeInSectors;
    
    // Limita pelo tamanho do bitmap (1 setor = 512 bytes = 4096 bits max)
    // Cada bit representa um bloco (1=ocupado, 0=livre)
    if (numBlocks > DISK_SECTORDATASIZE * 8) {
        numBlocks = DISK_SECTORDATASIZE * 8;
    }
    
    if (numBlocks == 0) return -1;
    
    // Preenche o superbloco com informacoes do sistema de arquivos
    superblock.magic = 0x4D595346; // Magic number "MYSF" para identificacao
    superblock.blockSize = blockSizeInSectors; // Tamanho do bloco em setores
    superblock.numBlocks = numBlocks; // Total de blocos de dados
    superblock.firstDataBlock = firstDataSector; // Onde comeca a area de dados
    superblock.numInodes = numInodes; // Maximo de arquivos/diretorios
    
    // Salva o superbloco no setor 0
    if (saveSuperblock(d) < 0) return -1;
    
    // Inicializa bitmap com zeros (todos os blocos livres)
    memset(bitmap, 0, DISK_SECTORDATASIZE);
    if (saveBitmap(d) < 0) return -1;
    
    // IMPORTANTE: Inicializa todos os inodes com inodeCreate
    // Isso garante que cada inode tenha seu campo 'number' correto
    // inodeFindFreeInode procura inodes com blockAddr[0]==0, mas precisa
    // que o campo 'number' esteja preenchido para funcionar corretamente
    for (unsigned int i = 1; i <= numInodes; i++) {
        Inode *inode = inodeCreate(i, d);
        if (!inode) return -1;
        free(inode);
    }
    
    // Configura o inode 1 como diretorio raiz
    // Todo sistema de arquivos Unix-like tem um diretorio raiz
    Inode *rootInode = inodeLoad(MYFS_ROOT_INODE, d);
    if (!rootInode) return -1;
    
    inodeSetFileType(rootInode, FILETYPE_DIR); // Tipo diretorio
    inodeSetFileSize(rootInode, 0); // Diretorio vazio inicialmente
    inodeSetRefCount(rootInode, 1); // Uma referencia (o proprio sistema)
    
    if (inodeSave(rootInode) < 0) {
        free(rootInode);
        return -1;
    }
    free(rootInode);
    
    return numBlocks;
}

//Funcao para montagem/desmontagem do sistema de arquivos, se possível.
//Na montagem (x=1) e' a chance de se fazer inicializacoes, como carregar
//o superbloco na memoria. Na desmontagem (x=0), quaisquer dados pendentes
//de gravacao devem ser persistidos no disco. Retorna um positivo se a
//montagem ou desmontagem foi bem sucedida ou, caso contrario, 0.
int myFSxMount (Disk *d, int x) {
    if (!d) return 0;
    
    if (x == 1) {
        // Montagem
        if (loadSuperblock(d) < 0) return 0;
        
        // Verifica magic number
        if (superblock.magic != 0x4D595346) return 0;
        
        if (loadBitmap(d) < 0) return 0;
        
        mountedDisk = d;
        initFDTable();
        
        return 1; // Montagem bem sucedida
    } else {
        // Desmontagem
        if (mountedDisk != d) return 0;
        
        // Fecha todos os descritores abertos deste disco
        for (int i = 0; i < MAX_FDS; i++) {
            if (fdTable[i].isOpen && fdTable[i].disk == d) {
                fdTable[i].isOpen = 0;
            }
        }
        
        // Persiste bitmap
        if (saveBitmap(d) < 0) return 0;
        
        mountedDisk = NULL;
        return 1; // Desmontagem bem sucedida
    }
}

//Funcao para abertura de um arquivo, a partir do caminho especificado
//em path, no disco montado especificado em d, no modo Read/Write,
//criando o arquivo se nao existir. Retorna um descritor de arquivo,
//em caso de sucesso. Retorna -1, caso contrario.
//
// DETALHES DA IMPLEMENTACAO:
// 1. Busca o arquivo no diretorio raiz
// 2. Se nao existir, cria novo arquivo (inode + entrada no diretorio)
// 3. Aloca descritor de arquivo (fd) e retorna
//
// ESTRUTURA DE DIRETORIO:
// Cada entrada tem 260 bytes: 4 bytes (numero do inode) + 256 bytes (nome)
// As entradas sao armazenadas sequencialmente nos blocos do diretorio
//
// CONVFASE 1: BUSCA O ARQUIVO NO DIRETORIO RAIZ
    // Carrega o inode do diretorio raiz (inode 1)
    Inode *rootInode = inodeLoad(MYFS_ROOT_INODE, d);
    if (!rootInode) return -1;
    
    unsigned int fileInodeNum = 0; // Sera preenchido se arquivo existir
    unsigned int fileSize = inodeGetFileSize(rootInode); // Tamanho do diretorio em bytes
    
    // Calcula tamanho de uma entrada de diretorio
    // Formato: [4 bytes: numero do inode][256 bytes: nome do arquivo + \0]
    unsigned int entrySize = sizeof(unsigned int) + MAX_FILENAME_LENGTH + 1;
    unsigned int numEntries = fileSize / entrySize; // Quantas entradas existem
    
    // Percorre todas as entradas do diretorio raiz buscando o arquivo
    if (strlen(filename) == 0 || strlen(filename) > MAX_FILENAME_LENGTH) return -1;
    
    // Carrega o inode raiz para buscar o arquivo
    Inode *rootInode = inodeLoad(MYFS_ROOT_INODE, d);
    if (!rootInode) return -1;
    
    unsigned int fileInodeNum = 0;
    unsigned int fileSize = inodeGetFileSize(rootInode);
    
    // Tamanho de uma entrada de diretorio: 4 bytes (inode) + 256 bytes (nome)
    unsigned int entrySize = sizeof(unsigned int) + MAX_FILENAME_LENGTH + 1;
    unsigned int numEntries = fileSize / entrySize;
    
    // B// Calcula onde esta a entrada i no disco
        unsigned int entryOffset = i * entrySize; // Offset em bytes desde inicio do diretorio
        unsigned int blockNum = entryOffset / (superblock.blockSize * DISK_SECTORDATASIZE); // Qual bloco
        unsigned int offsetInBlock = entryOffset % (superblock.blockSize * DISK_SECTORDATASIZE); // Offset no bloco
        
        // Obtem endereco do bloco no inode
        // CONVENCAO: inodes armazenam blockAddr+1 (0 significa "nao alocado")
        unsigned int blockAddrStored = inodeGetBlockAddr(rootInode, blockNum);
        if (blockAddrStored == 0) break; // Bloco nao alocado, fim do diretorio
        
        unsigned int blockAddr = blockAddrStored - 1;  // Converte para indice real (0-based)
        unsigned int sectorAddr = blockToSector(blockAddr); // Primeiro setor do bloco
        unsigned int sectorOffset = offsetInBlock / DISK_SECTORDATASIZE; // Qual setor dentro do bloco
        unsigned int byteOffset = offsetInBlock % DISK_SECTORDATASIZE; // Offset dentro do setor
        // Le o setor que contem a entrada
        unsigned char sector[DISK_SECTORDATASIZE];
        if (diskReadSector(d, sectorAddr + sectorOffset, sector) < 0) {
            free(rootInode);
            return -1;
        }
        
        // Extrai o numero do inode (primeiros 4 bytes da entrada)
        unsigned int entryInode;
        char2ul(&sector[byteOffset], &entryInode);
        
        // Extrai o nome do arquivo (proximos 256 bytes)
        char entryName[MAX_FILENAME_LENGTH + 1];
        memcpy(entryName, &sector[byteOffset + sizeof(unsigned int)], MAX_FILENAME_LENGTH + 1);
        
        // Compara com o nome procurado
        if (strcmp(entryName, filename) == 0) {
            fileInodeNum = entryInode; // Arquivo encontrado!
        }
    }
    
    // FASE 2: SE ARQUIVO NAO EXISTE, CRIA UM NOVO
    if (fileInodeNum == 0) {
        // 2.1: Aloca um inode livre para o novo arquivo
        // Comeca do inode 2 (inode 1 e' o diretorio raiz)
        fileInodeNum = inodeFindFreeInode(MYFS_ROOT_INODE + 1, d);
        if (fileInodeNum == 0) {
            free(rootInode);
            return -1; // Sem inodes disponiveis
        }
        
        // 2.2: Cria e inicializa o novo inode
        Inode *newInode = inodeCreate(fileInodeNum, d);
        if (!newInode) {
            free(rootInode);
            return -1;
        }
        
        inodeSetFileType(newInode, FILETYPE_REGULAR); // Arquivo regular (nao diretorio)
        inodeSetFileSize(newInode, 0); // Arquivo vazio
        inodeSetRefCount(newInode, 1); // Uma referencia (a entrada do diretorio)
        inodeSetFileType(newInode, FILETYPE_REGULAR);
        inodeSetFileSize(newInode, 0);
        inodeSetRefCount(newInode, 1);
        
        if (inodeSave(newInode) < 0) {
           2.3: Adiciona entrada do novo arquivo no diretorio raiz
        // A nova entrada vai no final do diretorio
        unsigned int newEntryOffset = fileSize; // fileSize = tamanho atual do diretorio
        unsigned int blockNum = newEntryOffset / (superblock.blockSize * DISK_SECTORDATASIZE);
        unsigned int offsetInBlock = newEntryOffset % (superblock.blockSize * DISK_SECTORDATASIZE);
        
        // Verifica se o diretorio ja tem um bloco alocado nesta posicao
        unsigned int blockAddrStored = inodeGetBlockAddr(rootInode, blockNum);
        unsigned int blockAddr;
        if (blockAddrStored == 0) {
            // Diretorio precisa de um novo bloco para a entrada
            unsigned int freeBlock = findFreeBlock();
            if (freeBlock == 0) {
                free(rootInode);
                return -1; // Disco cheio
            }
            blockAddr = freeBlock - 1;  // findFreeBlock retorna indice+1
            setBlockUsed(blockAddr); // Marca bloco como ocupado no bitmap
            if (saveBitmap(d) < 0) {
                free(rootInode);
                return -1;
            }
            // Adiciona bloco ao inode do diretorio
            // CONVENCAO: armazena blockAddr+1 (0 = nao alocado)
            if (inodeAddBlock(rootInode, blockAddr + 1) < 0) {
                free(rootInode);
                return -1;
            }
        } else {
            blockAddr = blockAddrStored - 1;  // Ja tem bloco, c= nao alocado)
            if (inodeAddBlock(rootInode, blockAddr + 1) < 0) {
                free(rootInode);
                return -1;
        // Calcula endereco do setor onde escrever a entrada
        unsigned int sectorAddr = blockToSector(blockAddr);
        unsigned int sectorOffset = offsetInBlock / DISK_SECTORDATASIZE;
        unsigned int byteOffset = offsetInBlock % DISK_SECTORDATASIZE;
        
        // Le o setor atual (pode ter outras entradas)
        unsigned char sector[DISK_SECTORDATASIZE];
        if (diskReadSector(d, sectorAddr + sectorOffset, sector) < 0) {
            free(rootInode);
            return -1;
        }
        
        // Escreve a entrada: [4 bytes: inode][256 bytes: nome]
        ul2char(fileInodeNum, &sector[byteOffset]); // Numero do inode
        
        // Escreve o nome do arquivo (preenche com zeros o restante)
        memset(&sector[byteOffset + sizeof(unsigned int)], 0, MAX_FILENAME_LENGTH + 1);
        strncpy((char*)&sector[byteOffset + sizeof(unsigned int)], filename, MAX_FILENAME_LENGTH);
        
        // Grava o setor modificado de volta
        if (diskWriteSector(d, sectorAddr + sectorOffset, sector) < 0) {
            free(rootInode);
            return -1;
        }
        
        // Atualiza tamanho do diretorio (agora tem mais uma entrada)
        inodeSetFileSize(rootInode, fileSize + entrySize);
        if (inodeSave(rootInode) < 0) {
            free(rootInode);
            return -1;
        }
    }
    
    free(rootInode);
    
    // FASE 3: ALOCA UM DESCRITOR DE ARQUIVO (FD)
    int fd = findFreeFD(); // Retorna valor de 1 a MAX_FDS
    if (fd < 0) return -1; // Tabela de FDs cheia
    
    // Configura o descritor de arquivo
    // IMPORTANTE: fd e' 1-based, mas fdTable e' 0-based
    int idx = fd - 1;
    fdTable[idx].isOpen = 1; // Marca como aberto
    fdTable[idx].inodeNumber = fileInodeNum; // Associa ao inode
    fdTable[idx].cursor = 0; // Cursor no inicio do arquivo
    fdTable[idx].disk = d; // Disco associado
    
    return fd; // Retorna descritor para o usuariox].isOpen = 1;
    fdTable[idx].inodeNumber = fileInodeNum;
    fdTable[idx].cursor = 0;
    fdTable[idx].disk = d;
    
    return fd;
}
	
//
// DETALHES DA IMPLEMENTACAO:
// 1. Valida fd e converte para indice da tabela (fd-1)
// 2. Calcula quantos bytes podem ser lidos (limita pelo tamanho do arquivo)
// 3. Le dados bloco por bloco, setor por setor
// 4. Atualiza cursor para proxima leitura
//
// ALGORITMO DE LEITURA:
// - Posiciona no byte atual (cursor)
// - Calcula qual bloco e offset dentro do bloco
// - Le setor(es) necessarios
// - Copia bytes para buffer
// - Repete ate ler nbytes ou chegar no fim do arquivo
int myFSRead (int fd, char *buf, unsigned int nbytes) {
    int idx = fd - 1;  // Converte fd (1-based) para indice (0-based)
    if (fd < 1 || fd > MAX_FDS || !fdTable[idx].isOpen || !buf) return -1;
    
    Disk *d = fdTable[idx].disk;
    if (!d) return -1;
    
    // Carrega inode do arquivo para obter metadados
    Inode *inode = inodeLoad(fdTable[idx].inodeNumber, d);
    if (!inode) return -1;
    
    unsigned int fileSize = inodeGetFileSize(inode);
    unsigned int cursor = fdTable[idx].cursor; // Posicao atual no arquivo
    
    // Verifica se ja esta no fim do arquivo
    // Loop de leitura: processa bytes ate completar nbytes ou chegar no fim
    while (bytesRead < bytesToRead) {
        // Calcula posicao atual no arquivo
        unsigned int currentPos = cursor + bytesRead;
        unsigned int blockNum = currentPos / blockDataSize; // Qual bloco logico
        unsigned int offsetInBlock = currentPos % blockDataSize; // Offset dentro do bloco
        
        // Obtem endereco fisico do bloco no disco
        unsigned int blockAddrStored = inodeGetBlockAddr(inode, blockNum);
        if (blockAddrStored == 0) break; // Bloco nao alocado (fim inesperado)
        
        unsigned int blockAddr = blockAddrStored - 1;  // Converte para indice real (0-based)
        unsigned int sectorAddr = blockToSector(blockAddr); // Primeiro setor do bloco
        unsigned int sectorOffset = offsetInBlock / DISK_SECTORDATASIZE; // Qual setor dentro do bloco
        unsigned int byteOffset = offsetInBlock % DISK_SECTORDATASIZE; // Offset dentro do setor
        
        // Le o setor que contem os dados
        unsigned char sector[DISK_SECTORDATASIZE];
        if (diskReadSector(d, sectorAddr + sectorOffset, sector) < 0) {
            free(inode);
            return -1;
        }
        
        // Calcula quantos bytes ler deste setor
        // (pode nao ler o setor inteiro se comecar no meio ou terminar antes do fim)
        unsigned int bytesInSector = DISK_SECTORDATASIZE - byteOffset;
        if (bytesInSector > bytesToRead - bytesRead) {
            bytesInSector = bytesToRead - bytesRead; // Limita ao necessario
        }
        
        // Copia bytes do setor para o buffer do usuario
        memcpy(&buf[bytesRead], &sector[byteOffset], bytesInSector);
        bytesRead += bytesInSector;
    }
    
    // Atualiza cursor para proxima leitura/escrita
    fdTable[idx].cursor += bytesRead;
    free(inode);
    
    return bytesRead; // Retorna quantos bytes foram efetivamente lidos
        unsigned char sector[DISK_SECTORDATASIZE];
        if (diskReadSector(d, sectorAddr + sectorOffset, sector) < 0) {
            free(inode);
//
// DETALHES DA IMPLEMENTACAO:
// 1. Valida fd e converte para indice da tabela
// 2. Para cada posicao a escrever:
//    - Verifica se bloco existe, senao aloca um novo
//    - Le setor, modifica bytes necessarios, escreve de volta
// 3. Atualiza cursor e tamanho do arquivo se cresceu
//
// ALOCACAO DINAMICA:
// Blocos sao alocados sob demanda (lazy allocation)
// Se escrever na posicao X e nao houver bloco, aloca automaticamente
int myFSWrite (int fd, const char *buf, unsigned int nbytes) {
    int idx = fd - 1;  // Converte fd (1-based) para indice (0-based)
    if (fd < 1 || fd > MAX_FDS || !fdTable[idx].isOpen || !buf) return -1;
    
    Disk *d = fdTable[idx].disk;
    if (!d) return -1;
    
    // Carrega inode do arquivo
    // Loop de escrita: processa bytes ate completar nbytes ou disco cheio
    while (bytesWritten < nbytes) {
        // Calcula posicao atual no arquivo
        unsigned int currentPos = cursor + bytesWritten;
        unsigned int blockNum = currentPos / blockDataSize; // Qual bloco logico
        unsigned int offsetInBlock = currentPos % blockDataSize; // Offset dentro do bloco
        
        // Verifica se o bloco ja esta alocado
        unsigned int blockAddrStored = inodeGetBlockAddr(inode, blockNum);
        unsigned int blockAddr;
        
        // ALOCACAO DINAMICA: Se bloco nao existe, aloca um novo
        if (blockAddrStored == 0) {
            unsigned int freeBlock = findFreeBlock(); // Retorna indice+1 ou 0
            if (freeBlock == 0) {
                // Disco cheio - para a escrita aqui
                break;
            }
            blockAddr = freeBlock - 1;  // Converte para indice real (0-based)
            
            // Marca bloco como ocupado no bitmap
            setBlockUsed(blockAddr);
            if (saveBitmap(d) < 0) {
                free(inode);
                return -1;
            }
            
            // Adiciona bloco ao inode do arquivo
            // CONVENCAO: armazena blockAddr+1 (0 = nao alocado)
            if (inodeAddBlock(inode, blockAddr + 1) < 0) {
                free(inode);
                return -1;
            }
            
        // Calcula endereco do setor onde escrever
        unsigned int sectorAddr = blockToSector(blockAddr);
        unsigned int sectorOffset = offsetInBlock / DISK_SECTORDATASIZE;
        unsigned int byteOffset = offsetInBlock % DISK_SECTORDATASIZE;
        
        // IMPORTANTE: Le o setor antes de escrever (read-modify-write)
        // Pode estar escrevendo no meio de um setor com dados existentes
        unsigned char sector[DISK_SECTORDATASIZE];
        if (diskReadSector(d, sectorAddr + sectorOffset, sector) < 0) {
            free(inode);
            return -1;
        }
        
        // Calcula quantos bytes escrever neste setor
        unsigned int bytesInSector = DISK_SECTORDATASIZE - byteOffset;
        if (bytesInSector > nbytes - bytesWritten) {
            bytesInSector = nbytes - bytesWritten;
        }
        
        // Copia dados do buffer do usuario para o setor
        memcpy(&sector[byteOffset], &buf[bytesWritten], bytesInSector);
        
        // Escreve setor modificado de volta para o disco
        if (diskWriteSector(d, sectorAddr + sectorOffset, sector) < 0) {
            free(inode);
            return -1;
        }
        
        bytesWritten += bytesInSector;
    }
    
    // Atualiza cursor para proxima operacao
    fdTable[idx].cursor += bytesWritten;
    
    // Se escreveu alem do tamanho atual, atualiza tamanho do arquivo
    if (fdTable[idx].cursor > fileSize) {
        inodeSetFileSize(inode, fdTable[idx].cursor);
        if (inodeSave(inode) < 0) {
            free(inode);
            return -1;
        }
    }
    
    free(inode);
    return bytesWritten; // Retorna quantos bytes foram efetivamente escritosblockAddrStored - 1;  // Converte para indice real
        }
        
        unsigned int sectorAddr = blockToSector(blockAddr);
        unsigned int sectorOffset = offsetInBlock / DISK_SECTORDATASIZE;
        unsigned int byteOffset = offsetInBlock % DISK_SECTORDATASIZE;
        
        unsigned char sector[DISK_SECTORDATASIZE];
        if (diskReadSector(d, sectorAddr + sectorOffset, sector) < 0) {
            free(inode);
            return -1;
        }
        
        // Calcula quantos bytes escrever neste setor
        unsigned int bytesInSector = DISK_SECTORDATASIZE - byteOffset;
        if (bytesInSector > nbytes - bytesWritten) {
            bytesInSector = nbytes - bytesWritten;
        }
        
        memcpy(&sector[byteOffset], &buf[bytesWritten], bytesInSector);
        
        if (diskWriteSector(d, sectorAddr + sectorOffset, sector) < 0) {
            free(inode);
            return -1;
        }
        
        bytesWritten += bytesInSector;
    }
    
    // Atualiza cursor
    fdTable[idx].cursor += bytesWritten;
    
    // Atualiza tamanho do arquivo se necessario
    if (fdTable[idx].cursor > fileSize) {
        inodeSetFileSize(inode, fdTable[idx].cursor);
        if (inodeSave(inode) < 0) {
            free(inode);
            return -1;
        }
    }
    
    free(inode);
    return bytesWritten;
}

//Funcao para fechar um arquivo, a partir de um descritor de arquivo
//existente. Retorna 0 caso bem sucedido, ou -1 caso contrario
int myFSClose (int fd) {
    int idx = fd - 1;  // Converte fd (1-based) para indice (0-based)
    if (fd < 1 || fd > MAX_FDS || !fdTable[idx].isOpen) return -1;
    
    fdTable[idx].isOpen = 0;
    fdTable[idx].inodeNumber = 0;
    fdTable[idx].cursor = 0;
    fdTable[idx].disk = NULL;
    
    return 0;
}

//Funcao para abertura de um diretorio, a partir do caminho
//especificado em path, no disco indicado por d, no modo Read/Write,
//criando o diretorio se nao existir. Retorna um descritor de arquivo,
//em caso de sucesso. Retorna -1, caso contrario.
int myFSOpenDir (Disk *d, const char *path) {
	return -1;
}

//Funcao para a leitura de um diretorio, identificado por um descritor
//de arquivo existente. Os dados lidos correspondem a uma entrada de
//diretorio na posicao atual do cursor no diretorio. O nome da entrada
//e' copiado para filename, como uma string terminada em \0 (max 255+1).
//O numero do inode correspondente 'a entrada e' copiado para inumber.
//Retorna 1 se uma entrada foi lida, 0 se fim de diretorio ou -1 caso
//mal sucedido
int myFSReadDir (int fd, char *filename, unsigned int *inumber) {
	return -1;
}

//Funcao para adicionar uma entrada a um diretorio, identificado por um
//descritor de arquivo existente. A nova entrada tera' o nome indicado
//por filename e apontara' para o numero de i-node indicado por inumber.
//Retorna 0 caso bem sucedido, ou -1 caso contrario.
int myFSLink (int fd, const char *filename, unsigned int inumber) {
	return -1;
}

//Funcao para remover uma entrada existente em um diretorio, 
//identificado por um descritor de arquivo existente. A entrada e'
//identificada pelo nome indicado em filename. Retorna 0 caso bem
//sucedido, ou -1 caso contrario.
int myFSUnlink (int fd, const char *filename) {
	return -1;
}

//Funcao para fechar um diretorio, identificado por um descritor de
//arquivo existente. Retorna 0 caso bem sucedido, ou -1 caso contrario.	
int myFSCloseDir (int fd) {
	return -1;
}

//Funcao para instalar seu sistema de arquivos no S.O., registrando-o junto
//ao virtual FS (vfs). Retorna um identificador unico (slot), caso
//o sistema de arquivos tenha sido registrado com sucesso.
//Caso contrario, retorna -1
int installMyFS (void) {
    // Aloca e preenche a estrutura FSInfo
    static FSInfo myfsInfo;
    
    myfsInfo.fsid = MYFS_ID;
    myfsInfo.fsname = "MyFS";
    myfsInfo.isidleFn = myFSIsIdle;
    myfsInfo.formatFn = myFSFormat;
    myfsInfo.xMountFn = myFSxMount;
    myfsInfo.openFn = myFSOpen;
    myfsInfo.readFn = myFSRead;
    myfsInfo.writeFn = myFSWrite;
    myfsInfo.closeFn = myFSClose;
    myfsInfo.opendirFn = myFSOpenDir;
    myfsInfo.readdirFn = myFSReadDir;
    myfsInfo.linkFn = myFSLink;
    myfsInfo.unlinkFn = myFSUnlink;
    myfsInfo.closedirFn = myFSCloseDir;
    
    // Inicializa a tabela de descritores
    initFDTable();
    
    // Registra o sistema de arquivos no VFS
    return vfsRegisterFS(&myfsInfo);
}
