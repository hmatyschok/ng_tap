<pre><code> 
ng_tap(4) - Service Access Point [SAP] for Terminal AccesS Point [TAP]
======================================================================

 Basic implementation of a SAP for Network TAP as extension for  
 the Implementation of device-driver for Ethernet NICs those are
 capable receiving and passing the trailer of Ethernet frames.
      
 The code-base of the device driver in sys/dev was extented by 
 utilizing the netgraph(4) kernel programmers interface. It's 
 an object oriented framework for implementing messege-passing 
 within and between nodes of netgraph(4) protocol domain(9). 
 
 The implementation relies on so called boilerplate code as 
 defined as preprocessor macros in netgraph/ng_tap.h.
 
Legal Notice: 
-------------
 
  (a) FreeBSD is a trademark of the FreeBSD Foundation. 
  
  (b) Don't use this software on production systems!
  
</code></pre>

