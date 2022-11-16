#ifndef __NETLINK_WRAP_H
#define __NETLINK_WRAP_H

#ifdef WINXX
#pragma warning (push)
#pragma warning (disable: 4200) // zero-sized array in struct/union
#include "linux/rtnetlink.h"
#pragma warning (pop)
#else
#include <errno.h>
#include <sys/socket.h>
#include <linux/rtnetlink.h>
#endif
#include <string>

namespace netlink
{

template< typename Payload, size_t AttrsLen = 0 >
struct pld_type
{
  Payload  pld;
  char attrs[NLMSG_ALIGN(AttrsLen)];

  operator Payload& () { return pld; }
};

template< typename Payload >
struct pld_type< Payload, 0 >
{
  Payload  pld;
  operator Payload& () { return pld; }
};

template< size_t AttrsLen >
struct attrs_type
{
  nlmsghdr& hdr;
  attrs_type(nlmsghdr& _hdr) : hdr(_hdr) {}

  void add(uint16_t _type, void* _data, uint16_t _len) {
    void* a = add_(_type, _len);
    MEMCPY(a, _data, _len);
  }

  template< typename T >
  void add(uint16_t _type, const T& _data) {
    add(_type, (void*)&_data, (uint16_t)sizeof(T));
  }
private:
  void* add_(uint16_t _type, uint16_t _len) {
    rtattr* a = (rtattr*)((char*)&hdr + NLMSG_ALIGN(hdr.nlmsg_len));
    a->rta_len = RTA_LENGTH(_len);
    a->rta_type = _type;
    hdr.nlmsg_len = NLMSG_ALIGN(hdr.nlmsg_len) + RTA_ALIGN(a->rta_len);
    return RTA_DATA(a);
  }
};

template<>
struct attrs_type<0>
{
  attrs_type(nlmsghdr& _hdr) {}
};


template< typename Payload, size_t AttrsLen = 0 >
struct request
{
  nlmsghdr hdr;
  pld_type<Payload, AttrsLen> m_pld;
  //TYPEDEF_ASSERT( NLMSG_ALIGN(sizeof(nlmsghdr)) == offsetof(m_pld) );

  request( uint16_t _type, uint16_t _flags = NLM_F_REQUEST )
  {
    memset(&m_pld, 0, sizeof(m_pld));
    hdr.nlmsg_len = NLMSG_LENGTH(sizeof(Payload));
    hdr.nlmsg_type = _type;
    hdr.nlmsg_flags = _flags;
    //hdr.nlmsg_seq = _seqnum; // Sequence number
    //hdr.nlmsg_pid = _pid; // Sender port ID
  }

  //nlmsghdr& hdr() { return hdr; }
  Payload& pld() { return m_pld; }
  attrs_type<AttrsLen> attrs() { return attrs_type<AttrsLen>(hdr); }

  void* add_attr_( uint16_t _type, size_t _len )
  {
    rtattr* a = (rtattr*)((char*)&hdr + NLMSG_ALIGN(hdr.nlmsg_len));
    a->rta_len = (uint16_t)RTA_LENGTH(_len);
    a->rta_type = _type;
    hdr.nlmsg_len = NLMSG_ALIGN(hdr.nlmsg_len) + RTA_ALIGN(a->rta_len);
    return RTA_DATA(a);
  }

  template< typename T >
  T* add_attr( uint16_t _type, size_t _len = 0 ) { return (T*)add_attr_( _type, (uint16_t)(sizeof(T) + _len) ); }

  void set_attr( uint16_t _type, void* _data, size_t _len ) {
    void* a = add_attr_( _type, _len );
    MEMCPY( a, _data, _len );
  }

  template< typename T >
  void set_attr( uint16_t _type, const T& _data ) {
    set_attr( _type, (void*)&_data, (uint16_t)sizeof(T) );
  }
};

template<>
struct request<void, 0>
{
  nlmsghdr hdr;
  request( uint16_t _type, uint16_t _flags = NLM_F_REQUEST )
  {
    hdr.nlmsg_len = NLMSG_LENGTH(0);
    hdr.nlmsg_type = _type;
    hdr.nlmsg_flags = _flags;
    //hdr.nlmsg_seq = _seqnum; // Sequence number
    //hdr.nlmsg_pid = _pid; // Sender port ID
  }
};


template< size_t N >
struct response
{
  ssize_t len;
  union {
    struct nlmsghdr hdr;
    char buf[N];
  } data;

  response() : len(0) {}
};

struct rtattr_iterator
{
  const rtattr* attr;
  int  len; // total length of rest attrs

  bool end() const {
    return !RTA_OK(attr, len);
  }
  rtattr_iterator& operator ++() {
    attr = RTA_NEXT(attr, len);
    return *this;
  }
  const rtattr* operator ->() const {
    return attr;
  }
  unsigned short type() const {
    return attr->rta_type;
  }
  void* data() const {
    return RTA_DATA(attr);
  }
  int   datalen() const {
    return RTA_PAYLOAD(attr);
  }
  template< typename T >
  bool copyto(T& _t) const {
    ASSERT((int)sizeof(T) == len);
    if((int)sizeof(T) != len )
      return false;
    MEMCPY((void*)&_t, data(), (unsigned)len);
    return true;
  }
};

template< uint16_t MsgType = 0 >
struct msg_type { static bool check(const nlmsghdr* msg) { return msg->nlmsg_type == MsgType; } };

template<>
struct msg_type<0> { static bool check(const nlmsghdr* msg) { return msg->nlmsg_type != NLMSG_NOOP; } };
template<>
struct msg_type<NLMSG_NOOP>;

template< uint16_t MsgType >
struct msg_iterator_base
{
  const nlmsghdr* msg;
  int  len;

  template< size_t N >
  msg_iterator_base(response<N>& _resp) : msg(&_resp.data.hdr), len((int)_resp.len) { ASSERT( NLMSG_ALIGN(msg->nlmsg_len) <= (unsigned)NLMSG_ALIGN(len) ); find(); }

  void find() {
    for( ; !end() && !msg_type<MsgType>::check(msg); ) {
      msg = NLMSG_NEXT(msg, len);
    }
  }

  void operator ++() {
    msg = NLMSG_NEXT(msg, len); find();
  }

  const nlmsghdr* operator ->() const {
    return msg;
  }

  bool end() const {
    return !NLMSG_OK(msg, len); // check NLMSG_DONE ???
  }
  uint16_t type() const {
    return msg->nlmsg_type;
  }
  /*bool done() const {
    return NLMSG_DONE == type();
  }*/
  int pldlen() const {
    return NLMSG_PAYLOAD(msg, 0); // nlmsg_len - <hdr len>
  }

  nlmsghdr* copy() const
  {
    ASSERT( msg->nlmsg_len <= (__u32)len );
    char* p = new char[msg->nlmsg_len]; // TODO align
    MEMCPY( p, msg, msg->nlmsg_len );
    return (nlmsghdr*)p;
  }
};

template< typename Payload >
struct msg_t
{
  const nlmsghdr* hdr;
  msg_t( const nlmsghdr* _hdr ) : hdr(_hdr) {}
  const Payload*  pld() const {  return (const Payload*)NLMSG_DATA(hdr); }
  rtattr_iterator attrs() const
  {
    rtattr_iterator it;
    it.attr = (const rtattr*)((const char*)pld() + NLMSG_ALIGN(sizeof(Payload)));
    it.len = NLMSG_PAYLOAD(hdr, 0) - NLMSG_ALIGN(sizeof(Payload)); ASSERT( NLMSG_PAYLOAD(hdr, sizeof(Payload)) == it.len );
    return it;
  }
};


template< uint16_t MsgType = 0, typename Payload = void >
struct msg_iterator;

template< class Payload >
struct msg_iterator< 0, Payload >; // forbiden


template<>
struct msg_iterator< 0, void > : msg_iterator_base<0>
{
  typedef msg_iterator_base<0> base_t;
  template< size_t N >
  msg_iterator(response<N>& _resp) : base_t(_resp) {}

  template< typename Payload >
  msg_t<Payload> get() const { return msg_t<Payload>(base_t::msg); }
};

template< uint16_t MsgType >
struct msg_iterator< MsgType, void > : msg_iterator_base<MsgType>
{
  typedef msg_iterator_base<MsgType> base_t;
  template< size_t N >
  msg_iterator(response<N>& _resp) : base_t(_resp) {}

  template< typename Payload >
  msg_t<Payload> get() const { return msg_t<Payload>(base_t::msg); }
};

template< uint16_t MsgType, class Payload >
struct msg_iterator : msg_iterator_base<MsgType>
{
  typedef msg_iterator_base<MsgType> base_t;
  template< size_t N >
  msg_iterator(response<N>& _resp) : base_t(_resp) {}

  msg_t<Payload> get() const { return msg_t<Payload>(base_t::msg); }
  const Payload* pld() const { return get().pld(); }
  rtattr_iterator attrs() const { return get().attrs(); }
};

#ifdef WINXX
static inline char* strerror_r( int err, char* buf, size_t size ) { strerror_s(buf, size, err); return buf; }
#endif

static inline void format_error_( std::string* _out, const char* _pref, int _err )
{
  *_out = _pref;
  //*_out += strerror(_err);
  char buf[1024];
  *_out += strerror_r( _err, buf, sizeof(buf) );
  SNPRINTF( buf, sizeof(buf), " (%d)", _err );
  *_out += buf;
}

static inline void get_last_error_( std::string* _out, const char* _pref )
{
  format_error_( _out, _pref, errno );
}

struct cmdsocket
{
#ifdef WINXX
  typedef SOCKET handle_t;
  static inline handle_t INVALID_HANDLE() { return INVALID_SOCKET; }
#else
  typedef int    handle_t;
  static inline handle_t INVALID_HANDLE() { return -1; }
#endif
  handle_t    handle;
  uint32_t    nlmsg_seq;    /* Sequence number */
  uint32_t    nlmsg_pid;    /* Sender port ID */
  std::string lasterr;

  cmdsocket() : handle(INVALID_HANDLE()), nlmsg_seq(0) {}
  ~cmdsocket()
  {
#ifndef WINXX
    if( INVALID_HANDLE() !=  handle )
      ::close(handle);
#endif
  }

  handle_t steal_handle()
  {
    handle_t r = handle;
    handle = INVALID_HANDLE();
    return r;
  }

  bool open( int _protocol = NETLINK_ROUTE, unsigned _nl_groups = 0 )
  {
#ifndef WINXX
    handle = ::socket(AF_NETLINK, SOCK_RAW, _protocol);
    if( INVALID_HANDLE() == handle ) {
      get_last_error_( &lasterr, "socket() " );
      return false;
    }
    struct sockaddr_nl addr;
    MEMZERO( &addr, sizeof(addr) );
    addr.nl_family = AF_NETLINK; /* fill-in kernel address (destination of our message) */
    addr.nl_groups = _nl_groups;
    //addr.nl_pid;     /* Port ID */ // If the application sets it to 0, the kernel takes care of assigning it.
    if( ::bind(handle, (struct sockaddr*)&addr, sizeof(addr)) ) {
      get_last_error_( &lasterr, "bind() " );
      ::close(handle);
      handle = INVALID_HANDLE();
      return false;
    }
    socklen_t sz = sizeof(addr);
    getsockname( handle, (struct sockaddr*)&addr, &sz );
    ASSERT( 0 != addr.nl_pid );
    nlmsg_pid = addr.nl_pid;
#endif
    return true;
  }

  const std::string& last_error() const { return lasterr; }
  bool was_error() const { return !lasterr.empty(); }

  bool send( const struct nlmsghdr& hdr )
  {
    struct sockaddr_nl addr;
    MEMZERO( &addr, sizeof(addr) );
    addr.nl_family = AF_NETLINK;
    ASSERT( 0 != hdr.nlmsg_len );
    const_cast<struct nlmsghdr&>(hdr).nlmsg_pid = this->nlmsg_pid;
    const_cast<struct nlmsghdr&>(hdr).nlmsg_seq = this->nlmsg_seq++;
    for( ;; )
    {
      ssize_t res = ::sendto(handle, (const char*)&hdr, hdr.nlmsg_len, 0, (sockaddr*)(&addr), sizeof(addr));
      if( res > 0 && (unsigned)res == hdr.nlmsg_len )
        break;
      if (errno == EINTR)
        continue;
      get_last_error_( &lasterr, "sendto() " );
      return false;
    }
    return true;
  }

  template< class T >
  bool send( const T& _request )
  {
    return send( _request.hdr );
  }

  template< class T >
  bool read( T& _response )
  {
    _response.len = ::recv(handle, (char*)&_response.data, sizeof(_response.data), MSG_DONTWAIT);
    if( _response.len < 0 || sizeof(_response.data) < _response.len) {
      get_last_error_( &lasterr, "recv() " );
      return false;
    }
    ASSERT( _response.len >= _response.data.hdr.nlmsg_len );
    return true;
  }

  template< class T >
  const nlmsgerr* read_ack( T& _response )
  {
    if( !read( _response ) )
      return NULL;
    msg_iterator<NLMSG_ERROR, nlmsgerr> msg( _response );
    if( msg.end() ) {
      lasterr = "NO_ACK";
      return NULL; // no ack
    }
    const nlmsgerr* err = msg.pld();
    return err; // 0 == err->error if ok
  }

  template< class T >
  bool send_read_ack( const T& _request )
  {
    if( !this->send( _request ) ) {
      ASSERT( !lasterr.empty() );
      return false;
    }
    response<1024> resp;
    const nlmsgerr* err = this->read_ack( resp );
    if( NULL == err ) {
      ASSERT( !lasterr.empty() );
      return false; // no ack
    }
    ASSERT( lasterr.empty() );
    if( err->error ) {
      format_error_( &lasterr, "NLMSG_ERROR ", -err->error );
      return false;
    }
    return true;
  }
};


} // namespace netlink

#endif // #ifndef __NETLINK_WRAP_H
