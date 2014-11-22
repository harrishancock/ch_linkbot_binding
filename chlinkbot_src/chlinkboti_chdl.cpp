#include"../include/barobo/linkbot.hpp"
#include<ch.h>
#include<stdio.h>

/*class creator*/
EXPORTCH void CLinkbotI_CLinkbotI_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    //class Linkbot *linkbot;
    char *id;
    printf("inside the dl file\n"); 
    Ch_VaStart(interp, ap, varg);
    
    id = Ch_VaArg(interp, ap, char *);
    printf("id %s\n", id);
    Linkbot l(id);
    printf("here\n");
    //Ch_CppChangeThisPointer(interp, linkbot, sizeof(Linkbot));
    Ch_VaEnd(interp, ap);
    return;
}
 
/* class destructor*/   
EXPORTCH void CLinkbotI_dCLinkbotI_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class Linkbot *linkbot;
    
    Ch_VaStart(interp, ap, varg);
    
    linkbot=Ch_VaArg(interp, ap, class Linkbot *);
    linkbot->~Linkbot();
    Ch_VaEnd(interp, ap);
    return;
}

/*linkbot connect*/
EXPORTCH void CLinkbotI_connect_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class Linkbot *linkbot;
    
    Ch_VaStart(interp, ap, varg);
    
    linkbot=Ch_VaArg(interp, ap, class Linkbot *);
    linkbot->connect();
    Ch_VaEnd(interp, ap);
    return;
}

/*linkbot move*/
EXPORTCH void CLinkbotI_move_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class Linkbot *linkbot;
    double j1;
    double j2;
    double j3;
    
    Ch_VaStart(interp, ap, varg);
    
    linkbot=Ch_VaArg(interp, ap, class Linkbot *);
    j1=Ch_VaArg(interp, ap, double);
    j2=Ch_VaArg(interp, ap, double);
    j3=Ch_VaArg(interp, ap, double);
    linkbot->move(j1, j2, j3);
    Ch_VaEnd(interp, ap);
    return;
}

/*linkbot move non blocking*/
EXPORTCH void CLinkbotI_moveNB_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class Linkbot *linkbot;
    double j1;
    double j2;
    double j3;
    
    Ch_VaStart(interp, ap, varg);
    
    linkbot=Ch_VaArg(interp, ap, class Linkbot *);
    j1=Ch_VaArg(interp, ap, double);
    j2=Ch_VaArg(interp, ap, double);
    j3=Ch_VaArg(interp, ap, double);
    linkbot->moveNB(j1, j2, j3);
    Ch_VaEnd(interp, ap);
    return;
}
