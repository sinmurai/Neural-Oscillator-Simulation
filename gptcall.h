/*                                               [gptcall.h]
 *
 * �ä�gnuplot���ñ������Ǥ���褦�ˤ���ѥå�������ɤ�
 *
 *                 G P T C A L L  version 0.32
 *
 *                Aug.19,2000  Presented by Chisato Yamauchi
 */
extern int gptopen( char * ) ;
extern int gptsend( int, const char *, ... ) ;
extern int gptclose( int ) ;
extern void gptcloseall( void ) ;
extern int vspawnvp( const char *, ... ) ;

