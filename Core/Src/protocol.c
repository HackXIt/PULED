/* PROTOCOL.c
 *   by Nikolaus Rieder, Stephan Gruber
 *
 * Created:
 *   12/5/2021, 2:07:57 PM
 * Last edited:
 *   12/5/2021, 7:31:07 PM
 * Auto updated?
 *   Yes
 *
 * Description:
 *   Code for custom communication protocol
**/

/*--- COMMON LIBRARIES ---*/
#include <stdlib.h>

/*--- CUSTOM LIBRARIES ---*/
#include "protocol.h"

/*--- MACROS ---*/
#define MAX_PARAMS 8 // We'll never have functions with more than 8 parameters

// NOTE: calloc() initializes all values to NULL or 0

/* Debug-Messages written to UART when using Protocol Functions
Out-of-Memory Errors:
    - generate_message()
    - generate_content()
    - add_item()
Successful execution:
    - clear_message()
    - clear_content()
*/

MSG_t *generate_message(uint8_t *byteArray)
{
    MSG_t *new_msg = (MSG_t *)calloc(1, sizeof(MSG_t));
    if (new_msg == NULL)
    {
        // TODO Write Error-Message to UART for generate_message()
        return NULL;
    }
    new_msg->function = (*byteArray >> 0) & 0xFF; // Get first byte for opcode
    size_t bytes = sizeof(byteArray);
    if (bytes == 1)
    { // No parameters in byteArray, only opcode - eg. ACK, NACK, ...
        return;
    }
    parse_byteArray(msg->content, byteArray);
    return new_msg;
}
list_t *generate_content()
{
    list_t *new_list = (list_t *)calloc(1, sizeof(list_t));
    if (new_list == NULL)
    {
        // TODO Write Error-Message to UART for generate_content()
        return NULL;
    }
    else
    {
        return new_list;
    }
}
void parse_byteArray(list_t *content, uint8_t *byteArray)
{
    /*
    This function should read through the byteArray and seperate it at the delimiter.
    Each read parameter should be allocated as a new variable and that's our item-pointer.
    Each seperation should add an item to content.
    I don't know how to do this yet at the moment.
    */
}
link_t *create_item(void *item)
{
    link_t *new_item = (link_t *)calloc(1, sizeof(link_t));
    if (new_item == NULL)
    {
        return NULL;
    }
    else
    {
        new_item->item = item;
    }
}
link_t *add_item(list *content, void *item)
{
    if (content == NULL)
    {
        // TODO Write Error-Message to UART for add_item()
        return NULL;
    }
    link_t *new_item = create_item(item);
    if (new_item == NULL)
        return NULL;
    if (content->head == NULL)
    { // First item in list
        content->head = new_item;
        content->count++;
    }
    else
    { // New item in list
        link_t *tmp = content->head;
        while (tmp->next != NULL)
        {
            tmp = tmp->next;
        }
        tmp->next = new_item;
        content->count++;
    }
}
void clear_message(MSG_t *msg)
{
    clear_content(msg->content);
    free(msg);
    // TODO Write Success-Message to UART for clear_message()
}
void clear_content(list_t *content)
{
    if (content == NULL)
        return;
    link_t *current = content->head;
    while (current != NULL)
    {
        content->head = current->next;
        free(current->item);
        free(current);
        current = content->head;
    }
    free(content);
    // TODO Write Success-Message to UART for clear_content()
    return;
}
// void *serialize_message(MSG_t *MSG_t);
// MSG_t *deserialize_message(uint8_t *byteArray);
