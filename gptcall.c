/*                                               [gptcall.c]
 *
 * Ｃでgnuplotを簡単に制御できるようにするパッケージもどき
 *
 *                 G P T C A L L  version 0.32
 *
 *                Aug.19,2000  Presented by Chisato Yamauchi
 */
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <stdarg.h>
#include <string.h>
#include <sys/types.h>
#include <sys/wait.h>
#define EXECFILE "gnuplot"  /* 起動するファイル */
#define EXECNUM  8          /* 起動するプロセスの最大個数の初期値 */
#define EXITMES  "exit\n"   /* gnuplot終了のコマンド */
#define VSPAWN   256        /* vspawnvpで最初に確保する文字サイズ */

struct gptbuf {
    FILE *sfp ;
    pid_t pid ;
    int sfds[2] ;
} ;
static struct gptbuf **tbladr ;
static int ttlnum = 0 ;

void gptcloseall( void ) ;

/* 新しくgnuplotを起動 */
/* 起動に成功したらコールナンバー, 失敗したら -1 を返す */
int gptopen( char *option )
{
    FILE *gfp ;
    int *pfds ;
    pid_t pid ;
    static int cmlc=0 ; 
    int i,cn ;
    static char *tmp1=NULL ;
    static char **args=NULL ;
    static int mlc=8 ;

    if( cmlc == 0 ){
        cmlc += EXECNUM ;
        tbladr = (struct gptbuf **)malloc(cmlc*sizeof(struct gptbuf *)) ;
        if( tbladr == NULL ) return(-1) ;
        for( i=cmlc-EXECNUM ; i<cmlc ; i++ ){
            tbladr[i] = NULL ;
        }
        if( atexit(&gptcloseall) ) return(-1) ;
    }
    for( cn=0 ; cn<cmlc ; cn++ ){
        if( tbladr[cn] == NULL ) break ;    /* 空きが見つかればそこを使用 */
    }
    if( cn==cmlc ){		/* 空きがない時 */
        cmlc += EXECNUM ;
        tbladr = (struct gptbuf **)realloc(tbladr,cmlc*sizeof(struct gptbuf *)) ;
        if( tbladr == NULL ) return(-1) ;
        for( i=cmlc-EXECNUM ; i<cmlc ; i++ ){
            tbladr[i] = NULL ;
        }
    }
    tbladr[cn] = (struct gptbuf *)malloc(sizeof(struct gptbuf)) ;
    if( tbladr[cn] == NULL ) return(-1) ;

    /* argsの作成 */
    if( tmp1!=NULL ) free(tmp1) ;
    tmp1=strdup(option) ;
    if( args==NULL ) args=(char **)malloc(sizeof(char *)*mlc) ;
    i=0 ;
    args[i++]=EXECFILE ;
    if( (args[i++]=strtok(tmp1," "))!=NULL ){
	while( (args[i++]=strtok(NULL," "))!=NULL ){
	    if( mlc==i ){
		mlc += 8 ;
		args=(char **)realloc(args,sizeof(char *)*mlc) ;
	    }
	}
    }

    pfds = tbladr[cn]->sfds ;
    if( pipe(pfds)<0 ) return(-1) ;
    if( ( pid=fork() ) < 0 ) exit(-1) ;
    if( pid == 0 ){
        dup2( pfds[0],0 ) ;
        close( pfds[1] ) ;
        close( pfds[0] ) ;
        execvp( *args, args ) ;
        _exit(-1) ;
    }
    close( pfds[0] ) ;
    gfp = fdopen( pfds[1],"w" ) ;
    if( gfp == NULL ) return(-1) ;
    tbladr[cn]->sfp = gfp ;
    tbladr[cn]->pid = pid ;
    if( ttlnum<cn ) ttlnum=cn ;
    return(cn) ;
}

int gptsend( int clnum, const char *format, ... )
{
    va_list ap ;
    if( ttlnum < clnum ) return(-1) ;
    if( clnum  < 0 ) return(-1) ;
    if( tbladr[clnum] == NULL ) return(-1) ;
    va_start( ap, format ) ;
    vfprintf( tbladr[clnum]->sfp, format, ap ) ;
    fflush( tbladr[clnum]->sfp ) ;
    /* vprintf(format,ap) ; */
    va_end(ap) ;
    return(0) ;
}

int gptclose( int clnum )
{
    int status ;
    if( ttlnum < clnum ) return(-1) ;
    if( clnum  < 0 ) return(-1) ;
    if( tbladr[clnum] == NULL ) return(-1) ;
    gptsend( clnum, EXITMES ) ;
    fclose( tbladr[clnum]->sfp ) ;
    close( (tbladr[clnum]->sfds)[1] ) ;
    waitpid(tbladr[clnum]->pid,&status,0) ;
    free( tbladr[clnum] ) ;
    tbladr[clnum] = NULL ;
    return(0) ;
}

void gptcloseall( void )
{
    int i ;
    for( i=0 ; i<=ttlnum ; i++ ){
        if( tbladr[i] != NULL ) gptclose( i ) ;
    }
}

int vspawnvp( const char *argsformat, ... )
{
    pid_t pid ;
    int status ;
    va_list ap ;
    char *adr ;
    char **argv ;
    char *args ;
    int siz = VSPAWN ;
    int i ;

    adr = (char *)malloc(sizeof(char)*siz) ;
    
    va_start( ap, argsformat ) ;
    do{
        i=vsnprintf( adr,siz,argsformat,ap ) ;
        if(i==-1){
            siz += VSPAWN ;
            adr = realloc(adr,sizeof(char)*siz) ;
        }
    } while(i==-1) ;
    va_end(ap) ;

    i=0 ;
    siz = VSPAWN ;
    argv = (char **)malloc(sizeof(char *)*siz) ;
    args = adr ;
    argv[i] = strtok(args," \n") ;
    while( argv[i]!=NULL ){
        if( siz<=++i ){
            siz += VSPAWN ;
            argv = realloc(argv,sizeof(char *)*siz) ;
        }
        argv[i] = strtok(NULL," \n") ;
    }
    if( ( pid = fork() ) < 0 ) return( -1 ) ;
    if( pid == 0 ){
        /* child process */
        execvp( *argv, argv ) ;
        _exit(1) ;
    }
    while(wait(&status)!=pid) ;
    free(argv) ;
    free(adr) ;
    /* wait! */
    if( WIFEXITED(status) ) return( 0 ) ;
    return( -1 ) ;
}

