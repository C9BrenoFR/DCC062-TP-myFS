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
int myFSFormat (Disk *d, unsigned int blockSize) {
    if (!d || blockSize == 0) return -1;
    
    unsigned long numSectors = diskGetNumSectors(d);
    if (numSectors < 4) return -1; // Precisa de pelo menos: superbloco, bitmap, inodes, dados
    
    // Calcula o tamanho do bloco em setores
    unsigned int blockSizeInSectors = blockSize / DISK_SECTORDATASIZE;
    if (blockSizeInSectors == 0) blockSizeInSectors = 1;
    
    // Calcula area de inodes (comeca no setor 2)
    unsigned int inodeAreaStart = inodeAreaBeginSector(); // Setor 2
    unsigned int inodesPerSector = inodeNumInodesPerSector();
    
    // Define numero maximo de inodes (usando alguns setores apos bitmap)
    // Reserva setores suficientes para ~64 inodes
    unsigned int inodeSectors = (64 + inodesPerSector - 1) / inodesPerSector;
    unsigned int numInodes = inodeSectors * inodesPerSector;
    
    // Primeiro setor de dados vem apos area de inodes
    unsigned int firstDataSector = inodeAreaStart + inodeSectors;
    
    // Calcula numero de blocos de dados disponiveis
    unsigned int dataSectors = numSectors - firstDataSector;
    unsigned int numBlocks = dataSectors / blockSizeInSectors;
    
    // Limita pelo tamanho do bitmap (1 setor = 512 bytes = 4096 bits)
    if (numBlocks > DISK_SECTORDATASIZE * 8) {
        numBlocks = DISK_SECTORDATASIZE * 8;
    }
    
    if (numBlocks == 0) return -1;
    
    // Preenche o superbloco
    superblock.magic = 0x4D595346; // "MYSF"
    superblock.blockSize = blockSizeInSectors;
    superblock.numBlocks = numBlocks;
    superblock.firstDataBlock = firstDataSector;
    superblock.numInodes = numInodes;
    
    // Salva o superbloco
    if (saveSuperblock(d) < 0) return -1;
    
    // Inicializa e salva o bitmap (todos os blocos livres)
    memset(bitmap, 0, DISK_SECTORDATASIZE);
    if (saveBitmap(d) < 0) return -1;
    
    // Inicializa todos os inodes (importante: inodeCreate coloca o number correto)
    for (unsigned int i = 1; i <= numInodes; i++) {
        Inode *inode = inodeCreate(i, d);
        if (!inode) return -1;
        free(inode);
    }
    
    // Configura o inode do diretorio raiz (inode 1)
    Inode *rootInode = inodeLoad(MYFS_ROOT_INODE, d);
    if (!rootInode) return -1;
    
    inodeSetFileType(rootInode, FILETYPE_DIR);
    inodeSetFileSize(rootInode, 0);
    inodeSetRefCount(rootInode, 1);
    
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
int myFSOpen (Disk *d, const char *path) {
    if (!d || !path) return -1;
    if (d != mountedDisk) return -1;
    
    // Remove a barra inicial se existir (simplificacao: so diretorio raiz)
    const char *filename = path;
    if (filename[0] == '/') filename++;
    
    if (strlen(filename) == 0 || strlen(filename) > MAX_FILENAME_LENGTH) return -1;
    
    // Carrega o inode raiz para buscar o arquivo
    Inode *rootInode = inodeLoad(MYFS_ROOT_INODE, d);
    if (!rootInode) return -1;
    
    unsigned int fileInodeNum = 0;
    unsigned int fileSize = inodeGetFileSize(rootInode);
    
    // Tamanho de uma entrada de diretorio: 4 bytes (inode) + 256 bytes (nome)
    unsigned int entrySize = sizeof(unsigned int) + MAX_FILENAME_LENGTH + 1;
    unsigned int numEntries = fileSize / entrySize;
    
    // Busca o arquivo no diretorio raiz
    for (unsigned int i = 0; i < numEntries && fileInodeNum == 0; i++) {
        unsigned int entryOffset = i * entrySize;
        unsigned int blockNum = entryOffset / (superblock.blockSize * DISK_SECTORDATASIZE);
        unsigned int offsetInBlock = entryOffset % (superblock.blockSize * DISK_SECTORDATASIZE);
        
        unsigned int blockAddrStored = inodeGetBlockAddr(rootInode, blockNum);
        if (blockAddrStored == 0) break;
        
        unsigned int blockAddr = blockAddrStored - 1;  // Converte para indice real
        unsigned int sectorAddr = blockToSector(blockAddr);
        unsigned int sectorOffset = offsetInBlock / DISK_SECTORDATASIZE;
        unsigned int byteOffset = offsetInBlock % DISK_SECTORDATASIZE;
        
        unsigned char sector[DISK_SECTORDATASIZE];
        if (diskReadSector(d, sectorAddr + sectorOffset, sector) < 0) {
            free(rootInode);
            return -1;
        }
        
        // Le o numero do inode
        unsigned int entryInode;
        char2ul(&sector[byteOffset], &entryInode);
        
        // Le o nome do arquivo
        char entryName[MAX_FILENAME_LENGTH + 1];
        memcpy(entryName, &sector[byteOffset + sizeof(unsigned int)], MAX_FILENAME_LENGTH + 1);
        
        if (strcmp(entryName, filename) == 0) {
            fileInodeNum = entryInode;
        }
    }
    
    // Se o arquivo nao existe, cria um novo
    if (fileInodeNum == 0) {
        // Encontra um inode livre
        fileInodeNum = inodeFindFreeInode(MYFS_ROOT_INODE + 1, d);
        if (fileInodeNum == 0) {
            free(rootInode);
            return -1;
        }
        
        // Cria o novo inode
        Inode *newInode = inodeCreate(fileInodeNum, d);
        if (!newInode) {
            free(rootInode);
            return -1;
        }
        
        inodeSetFileType(newInode, FILETYPE_REGULAR);
        inodeSetFileSize(newInode, 0);
        inodeSetRefCount(newInode, 1);
        
        if (inodeSave(newInode) < 0) {
            free(newInode);
            free(rootInode);
            return -1;
        }
        free(newInode);
        
        // Adiciona entrada no diretorio raiz
        unsigned int newEntryOffset = fileSize;
        unsigned int blockNum = newEntryOffset / (superblock.blockSize * DISK_SECTORDATASIZE);
        unsigned int offsetInBlock = newEntryOffset % (superblock.blockSize * DISK_SECTORDATASIZE);
        
        // Verifica se precisa de novo bloco
        unsigned int blockAddrStored = inodeGetBlockAddr(rootInode, blockNum);
        unsigned int blockAddr;
        if (blockAddrStored == 0) {
            // Aloca novo bloco
            unsigned int freeBlock = findFreeBlock();
            if (freeBlock == 0) {
                free(rootInode);
                return -1;
            }
            blockAddr = freeBlock - 1;  // findFreeBlock retorna indice + 1
            setBlockUsed(blockAddr);
            if (saveBitmap(d) < 0) {
                free(rootInode);
                return -1;
            }
            // Armazena blockAddr + 1 no inode (0 = nao alocado)
            if (inodeAddBlock(rootInode, blockAddr + 1) < 0) {
                free(rootInode);
                return -1;
            }
        } else {
            blockAddr = blockAddrStored - 1;  // Converte para indice real
        }
        
        unsigned int sectorAddr = blockToSector(blockAddr);
        unsigned int sectorOffset = offsetInBlock / DISK_SECTORDATASIZE;
        unsigned int byteOffset = offsetInBlock % DISK_SECTORDATASIZE;
        
        unsigned char sector[DISK_SECTORDATASIZE];
        if (diskReadSector(d, sectorAddr + sectorOffset, sector) < 0) {
            free(rootInode);
            return -1;
        }
        
        // Escreve o numero do inode
        ul2char(fileInodeNum, &sector[byteOffset]);
        
        // Escreve o nome do arquivo
        memset(&sector[byteOffset + sizeof(unsigned int)], 0, MAX_FILENAME_LENGTH + 1);
        strncpy((char*)&sector[byteOffset + sizeof(unsigned int)], filename, MAX_FILENAME_LENGTH);
        
        if (diskWriteSector(d, sectorAddr + sectorOffset, sector) < 0) {
            free(rootInode);
            return -1;
        }
        
        // Atualiza tamanho do diretorio raiz
        inodeSetFileSize(rootInode, fileSize + entrySize);
        if (inodeSave(rootInode) < 0) {
            free(rootInode);
            return -1;
        }
    }
    
    free(rootInode);
    
    // Encontra um descritor de arquivo livre
    int fd = findFreeFD();
    if (fd < 0) return -1;
    
    // Configura o descritor (fd-1 porque fd vai de 1 a MAX_FDS)
    int idx = fd - 1;
    fdTable[idx].isOpen = 1;
    fdTable[idx].inodeNumber = fileInodeNum;
    fdTable[idx].cursor = 0;
    fdTable[idx].disk = d;
    
    return fd;
}
	
//Funcao para a leitura de um arquivo, a partir de um descritor de arquivo
//existente. Os dados devem ser lidos a partir da posicao atual do cursor
//e copiados para buf. Terao tamanho maximo de nbytes. Ao fim, o cursor
//deve ter posicao atualizada para que a proxima operacao ocorra a partir
//do próximo byte apos o ultimo lido. Retorna o numero de bytes
//efetivamente lidos em caso de sucesso ou -1, caso contrario.
int myFSRead (int fd, char *buf, unsigned int nbytes) {
    int idx = fd - 1;  // Converte fd (1-based) para indice (0-based)
    if (fd < 1 || fd > MAX_FDS || !fdTable[idx].isOpen || !buf) return -1;
    
    Disk *d = fdTable[idx].disk;
    if (!d) return -1;
    
    Inode *inode = inodeLoad(fdTable[idx].inodeNumber, d);
    if (!inode) return -1;
    
    unsigned int fileSize = inodeGetFileSize(inode);
    unsigned int cursor = fdTable[idx].cursor;
    
    // Verifica se ja esta no fim do arquivo
    if (cursor >= fileSize) {
        free(inode);
        return 0;
    }
    
    // Calcula quantos bytes podem ser lidos
    unsigned int bytesToRead = nbytes;
    if (cursor + bytesToRead > fileSize) {
        bytesToRead = fileSize - cursor;
    }
    
    unsigned int bytesRead = 0;
    unsigned int blockDataSize = superblock.blockSize * DISK_SECTORDATASIZE;
    
    while (bytesRead < bytesToRead) {
        unsigned int currentPos = cursor + bytesRead;
        unsigned int blockNum = currentPos / blockDataSize;
        unsigned int offsetInBlock = currentPos % blockDataSize;
        
        unsigned int blockAddrStored = inodeGetBlockAddr(inode, blockNum);
        if (blockAddrStored == 0) break;
        
        unsigned int blockAddr = blockAddrStored - 1;  // Converte para indice real
        unsigned int sectorAddr = blockToSector(blockAddr);
        unsigned int sectorOffset = offsetInBlock / DISK_SECTORDATASIZE;
        unsigned int byteOffset = offsetInBlock % DISK_SECTORDATASIZE;
        
        unsigned char sector[DISK_SECTORDATASIZE];
        if (diskReadSector(d, sectorAddr + sectorOffset, sector) < 0) {
            free(inode);
            return -1;
        }
        
        // Calcula quantos bytes ler deste setor
        unsigned int bytesInSector = DISK_SECTORDATASIZE - byteOffset;
        if (bytesInSector > bytesToRead - bytesRead) {
            bytesInSector = bytesToRead - bytesRead;
        }
        
        memcpy(&buf[bytesRead], &sector[byteOffset], bytesInSector);
        bytesRead += bytesInSector;
    }
    
    fdTable[idx].cursor += bytesRead;
    free(inode);
    
    return bytesRead;
}

//Funcao para a escrita de um arquivo, a partir de um descritor de arquivo
//existente. Os dados de buf sao copiados para o disco a partir da posição
//atual do cursor e terao tamanho maximo de nbytes. Ao fim, o cursor deve
//ter posicao atualizada para que a proxima operacao ocorra a partir do
//proximo byte apos o ultimo escrito. Retorna o numero de bytes
//efetivamente escritos em caso de sucesso ou -1, caso contrario
int myFSWrite (int fd, const char *buf, unsigned int nbytes) {
    int idx = fd - 1;  // Converte fd (1-based) para indice (0-based)
    if (fd < 1 || fd > MAX_FDS || !fdTable[idx].isOpen || !buf) return -1;
    
    Disk *d = fdTable[idx].disk;
    if (!d) return -1;
    
    Inode *inode = inodeLoad(fdTable[idx].inodeNumber, d);
    if (!inode) return -1;
    
    unsigned int fileSize = inodeGetFileSize(inode);
    unsigned int cursor = fdTable[idx].cursor;
    unsigned int bytesWritten = 0;
    unsigned int blockDataSize = superblock.blockSize * DISK_SECTORDATASIZE;
    
    while (bytesWritten < nbytes) {
        unsigned int currentPos = cursor + bytesWritten;
        unsigned int blockNum = currentPos / blockDataSize;
        unsigned int offsetInBlock = currentPos % blockDataSize;
        
        unsigned int blockAddrStored = inodeGetBlockAddr(inode, blockNum);
        unsigned int blockAddr;
        
        // Se nao ha bloco alocado para esta posicao, aloca um novo
        if (blockAddrStored == 0) {
            unsigned int freeBlock = findFreeBlock();
            if (freeBlock == 0) {
                // Sem blocos livres
                break;
            }
            blockAddr = freeBlock - 1;  // findFreeBlock retorna indice + 1
            setBlockUsed(blockAddr);
            if (saveBitmap(d) < 0) {
                free(inode);
                return -1;
            }
            // Armazena blockAddr + 1 no inode (0 = nao alocado)
            if (inodeAddBlock(inode, blockAddr + 1) < 0) {
                free(inode);
                return -1;
            }
            // Recarrega o inode apos adicionar bloco
            free(inode);
            inode = inodeLoad(fdTable[idx].inodeNumber, d);
            if (!inode) return -1;
        } else {
            blockAddr = blockAddrStored - 1;  // Converte para indice real
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
