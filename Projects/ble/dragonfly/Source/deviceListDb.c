////////////////////////////////////////////////////////////////////////////////
// databaase manage functions
////////////////////////////////////////////////////////////////////////////////

// deviceDB table initializtioin
static gattAttribute_t gateAttrTable[] =
{
  {
    {}, /* type */
    ,                       /* permissions */
    ,                                      /* handle */
                       /* pValue */
  },
    // Characteristic Declaration
    {
      {},
      ,
      ,

    },
      // Characteristic Value "Data"
      {
        {},
        ,
        ,

      },
      // Characteristic configuration
      {
        {},
        ,
        ,

      },
    // Characteristic Declaration
    {
      {},
      ,
      ,

    },
      // Characteristic Value "Configuration"
      {
        {},
        ,
        ,

      },
     // Characteristic Declaration "Period"
    {
      {},
      ,
      ,

    },
      // Characteristic Value "Period"
      {
        {},
        ,
        ,

      },
};




// constants and defines
# define NODE_NAME_LENGHT     10
# define MAX_NODES            3
# define MAX_CONNECTIONS      3

// typedef

  typedef enum
  {
    NODE_IDLE = 0,
    NODE_CONECTED,
    NODE_DOWNLOADING,
    NODE_DISCONNECTED
  } nodeState_t;

  typedef enum
  {
    CONN_IDLE = 0,
    CONN_CONNECTED,
    NODE_CONNECTING
  } connState_t;

  // typedef for nodes
  typedef struct
  {
    nodeState_t state;
    uint8 id;
    uint8 type;
    uint8 name[NODE_NAME_LENGHT];
  } node_t;

  // typedef for connections
  typedef struct
  {
    connState_t state;
    uint8  handle;
    uint8  addr[B_ADDR_LEN];
    node_t node;
  } conn_t;

  // typedef for data base
  typedef struct
  {
    conn_t conn[MAX_CONNECTIONS];
    uint8 aD  = 0; // actual Device
    uint8 cDD = 0; // connected Devices
    uint8 rDD = 0; // registered Devices
  } deviceDB_t;

  // internal protected variables
  static deviceDB_t deviceDB;

  // calls type: deviceDB.conn[aD].node.name

// function prototipes
  uint8_t initDeviceDB();
  uint8_t reisterDeviceDB();
  uint8_t nextDevice();


  uint8_t clearDeviceDB()
  {
    deviceDB.aD  = 0;
    deviceDB.cDD = 0;
    deviceDB.rDD = 0;
    for (uint8 i = 0; i < MAX_CONNECTIONS; i++)
    {
      deviceDB.conn[i].state  = CONN_IDLE;
      deviceDB.conn[i].handle = i;
      deviceDB.conn[i].addr[B_ADDR_LEN] = {0x00,0x00,0x00,0x00,0x00,0x00};
    }

    return SUCCESS;
  }

  uint8_t initDeviceDB()
  {
    conn_t conn;

    deviceDB.aD  = 0;
    deviceDB.cDD = 0;
    deviceDB.rDD = 3;

    connState_t state;
    uint8  handle;
    uint8  addr[B_ADDR_LEN];
    node_t node;

    conn[0].state = CON_IDLE;
    conn[0].handle

    for (uint8 i = 0; i < MAX_CONNECTIONS; i++)
    {

      regDeviceDB(i, )
      deviceDB.conn[i].state  = CONN_IDLE;
      deviceDB.conn[i].handle = i;
      deviceDB.conn[i].addr[B_ADDR_LEN] = {0x00,0x00,0x00,0x00,0x00,0x00};
    }

    return SUCCESS;
  }


  uint8_t regDeviceDB(connNum, (conn_t *)connNew)
  {
    deviceDB.conn[connReg] = connNew;
    deviceDB.rDD++;
  }

//////////////////////////////////// a partir de aqui es test //////////////////
  uint8_t nextDevice() // prototipe

  uint8_t regDeviceDB(connNum, (conn_t *)connNew)
  {
    deviceDB.conn[connReg] = connNew;
    deviceDB.rDD++;
  }




  uint8_t regDeviceDB(connState_t conn)
  {
    if (once) {
      /* code */
    } else if (/* condition */) {
      /* code */
    } else {
      /* code */
    }


switch (control/* expression */) {
  case /* value */:
}

//que pasa pero ninio dime que te pasa
estabas enn tu casa y nno sabias nadaaaaaaaaaaaaa
que


que me tienes que decir???

  }
