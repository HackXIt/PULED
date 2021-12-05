/* PROTOCOL.h
 *   by Nikolaus Rieder, Stephan Gruber
 *
 * Created:
 *   12/5/2021, 2:07:27 PM
 * Last edited:
 *   12/5/2021, 6:20:05 PM
 * Auto updated?
 *   Yes
 *
 * Description:
 *   Definitions for custom communication protocol
**/

//---- Hex-Codes: 0x00 ... 0xFF
// 0x00 ... 0x0F - Success Codes
#define ACK 0x00
// 0xF0 ... 0xFF - Failure Codes
#define NACK 0xFF
// 0x10 ... 0xEF - Operation Codes in Total
// 0x?1 ... 0x?9 - Operation Codes of INPUT Module
#define INIT_INPUT_REQ 0x10   // Initiiert und startet das Input Module
#define INIT_INPUT_RES 0x11   // Response to INIT_INPUT
#define START_SAMPLE_REQ 0x12 // Startet das Sampling von Werten
#define START_SAMPLE_RES 0x13 // Response to START_SAMPLE
#define TEMP_ENABLE_REQ 0x20  // Enabled das Temperatur-Modul
#define TEMP_ENABLE_RES 0x21  // Response to TEMP_ENABLE
#define PULSE_REQ 0x30        // Fragt einen Pulswert an
#define PULSE_RES 0x31        // Response to PULSE - Gibt den Pulswert zurück
#define TEMP_REQ 0x32         // Fragt den Temperaturwert des Sensors an
#define TEMP_RES 0x33         // Response to TEMP - Gibt den Temperaturwert zurück
// 0x?A ... 0x?F - Operation Codes of OUTPUT Module
#define INIT_DISPLAY_REQ 0xA0 //Initialises the OLED display
#define INIT_DISPLAY_RES 0xA1 //Response after OLED display was initialised
#define PRINT_REQ 0xB0        // Prints text to the OLED display
#define PRINT_RES 0xB1        // Response to PRINT_REQ
#define DISPLAY_IMG_REQ 0xB2  //Displays the requested image
#define DISPLAY_IMG_RES 0xB3  //Response to DISPLAY_IMG_REQ
#define SET_TXT_COL_REQ 0xC0  //Sets the text color
#define SET_TXT_COL_RES 0xC1  //Response to SET_TXT_COL_REQ
#define SET_FG_COL_REQ 0xC2   //Sets foreground color(for images)
#define SET_FG_COL_RES 0xC3   //Response to SET_FG_COL_REQ
#define SET_BG_COL_REQ 0xC4   //Sets the background color of the OLED display
#define SET_BG_COL_RES 0xC5   //Response to SET_BG_COL_REQ
#define SET_FONT_REQ 0xC6     //Sets the Font on the OLEd display
#define SET_FONT_RES 0xC7     //Response to SET_FONT_REQ

//---- Message Definitions
#define SEP 0x7C   // Pipe-Symbol => |
#define START 0x7B // Curly-Braces => {
#define END 0x7D   // Curly-Braces => }

/* MESSAGE-AUFBAU Beispiel
START-Byte:
{
Message:
OP-CODE|PARAM_1|PARAM_2|...|PARAM_n
END-BYTE:
}
Example-Messages:
{0x00} => ACK
{0xFF} => NACK
{op-code|"Asdf"|1234|"asdf"} => Operation mit 3 Parametern
*/

// Anhand der OP-Codes ist klar definiert, wieviel Parameter und welche Parameter-Typen es gibt.
// ---- Message-Struktur
typedef struct protocol_msg
{
    char function;        // Op-Code
    struct list *content; // Dynamisch alloziiert
} MSG_t;

// NOTE: Jeder Parameter entspricht einem item in der Liste
typedef struct list
{
    struct link *head;
    int count;
} list_t;

typedef struct link
{
    struct link *next;
    void *item;
} link_t;

// ---- Protocol Functions
/************************************************
 * @brief Generates Message struct and indirectly initializes function-content with generate_Content()
 * 
 * @param byteArray The byteArray of the whole message content
 * @return MSG* 
 ***********************************************/
MSG_t *generate_message(uint8_t *byteArray);

/************************************************
 * @brief Initializes a new linked-list for function-content
 * 
 * @return list* 
 ***********************************************/
list_t *generate_content();

/************************************************
 * @brief Parses the given byteArray and sequentially adds them to the content-list
 * 
 * @param content the function-content linked-list where items are added
 * @param byteArray the byteArray to be parsed
 ***********************************************/
void parse_byteArray(list_t *content, uint8_t *byteArray);

/************************************************
 * @brief Create an item object
 * 
 * @param item reference to byteArray of item
 * @return link_t* 
 ***********************************************/
link_t *create_item(void *item);

/************************************************
 * @brief Adds an item to the function-content
 * 
 * @param content 
 * @param item 
 * @return link* 
 ***********************************************/
link_t *add_item(list_t *content, void *item);

// NOTE: Einzelne Items entfernen ist nicht implementiert, es muss immer die ganze Message verworfen werden!
/************************************************
 * @brief Clears the message from memory
 * 
 * @param msg 
 ***********************************************/
void clear_message(MSG_t *msg);

/************************************************
 * @brief Clears the function-content from memory
 * 
 * @param content 
 ***********************************************/
void clear_content(list_t *content);

/************************************************
 * @brief Serializes a message content into a sendable Byte-Array
 * 
 * @param msg 
 * @return uint8_t*
 ***********************************************/
uint8_t *serialize_message(MSG_t *msg);

/************************************************
 * @brief De-serializes a received Byte-Array into a new Message
 * 
 * @param byteArray 
 * @return MSG* 
 ***********************************************/
MSG_t *deserialize_message(uint8_t *byteArray);
