/*   $Id: AppErrors.h,v 1.3 1997/05/11 23:21:21 ags-sw Exp $  */
#ifndef APPERRORS_H
#define APPERRORS_H

enum {
        noErr                                           = 0
};

extern	void	DelayedExitToShell ( void );
extern	void	GestaltWhileInit ( uint32_t selector, int32_t *response );
extern	void	InitError ( char * string1, int32_t length1,
					char * string2, int32_t length2 );
extern	void	InitInfo ( char * string1, int32_t length1,
					char * string2, int32_t length2 );
extern	void	StartProgressBar ( void );
extern	void	EndProgressBar ( void );
extern	void	ProgressBar ( long double percentage );
extern	unsigned char	TrapAvailable ( short theTrap );

extern	void	StartupPtr ( char **theThing, size_t theSize,
					short errStrID, short errStrNo,
					void ( *CleanUp ) ( void ) );
extern	void	GenericErrorAlert ( short errStrID, short errStrNo,
					int32_t theError );
#endif

