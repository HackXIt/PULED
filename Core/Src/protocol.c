/* PROTOCOL.c
 *   by Nikolaus Rieder, Stephan Gruber
 *
 * Created:
 *   12/5/2021, 2:07:57 PM
 * Last edited:
 *   December 27, 2021, 10:45:07 PM GMT+1
 * Auto updated?
 *   Yes
 *
 * Description:
 *   Code for custom communication protocol
**/

/*--- COMMON LIBRARIES ---*/
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <stdio.h>

/*--- CUSTOM LIBRARIES ---*/
#include "protocol.h"

/*--- MACROS ---*/

// NOTE: calloc() initializes all values to NULL or 0

/* Debug-Messages written to UART when using Protocol Functions
Out-of-Memory Errors:
    - deserialize_message()
    - generate_content()
    - add_item()
Successful execution:
    - clear_message()
    - clear_content()
*/

MSG_t *deserialize_message(uint8_t *byteArray)
{
    MSG_t *new_msg = (MSG_t *)calloc(1, sizeof(MSG_t));
    if (new_msg == NULL)
    {
        // TODO Write Error-Message to UART for deserialize_message()
        return NULL;
    }
    new_msg->function = (*byteArray >> 0) & 0xFF; // Get first byte for opcode
    size_t bytes = sizeof(byteArray);
    if (bytes == 1)
    { // No parameters in byteArray, only opcode - eg. ACK, NACK, ...
        return new_msg;
    }
    new_msg->content = generate_content(byteArray);
    return new_msg;
}
list_t *generate_content(uint8_t *byteArray)
{
    list_t *new_list = (list_t *)calloc(1, sizeof(list_t));
    if (new_list == NULL)
    {
        // TODO Write Error-Message to UART for generate_content()
        return NULL;
    }
    else
    {
        parse_byteArray(new_list, (char *)byteArray);
        return new_list;
    }
}
void parse_byteArray(list_t *content, char *byteArray)
{
    char *token;
    char delimiter = (char)SEP;
    // NOTE https://stackoverflow.com/questions/16645583/how-to-copy-a-char-array-in-c
    size_t arraySize = strlen(byteArray) + 1;
    char *tmp_byteArray = (char *)calloc(arraySize, sizeof(char));
    snprintf(tmp_byteArray, arraySize, "%s", byteArray);
    token = strtok(tmp_byteArray + 2, &delimiter);
    while (token != NULL)
    {
        uint8_t status = add_item(content, token);
        switch (status)
        {
        case 0:
            token = strtok(NULL, &delimiter);
            break;
        case 1: // FIXME Content is NULL ??
            break;
        case 2: // TODO Write Out-Of-Memory Error-Message to UART for add_item status 2
            break;
        }
    }
    free(tmp_byteArray);
}
link_t *create_item(char *item)
{
    link_t *new_item = (link_t *)calloc(1, sizeof(link_t));
    if (new_item == NULL)
    {
        return NULL;
    }
    new_item->item = malloc(sizeof(char) * strlen(item) + 1);
    if (new_item->item == NULL)
    {
        free(new_item);
        return NULL;
    }
    strcpy(new_item->item, item);
    return new_item;
}
uint8_t add_item(list_t *content, char *item)
{
    if (content == NULL)
    {
        // TODO Write Error-Message to UART for add_item()
        return 1;
    }
    link_t *new_item = create_item(item);
    if (new_item == NULL)
        return 2;
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
    return 0;
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
uint8_t *serialize_message(MSG_t *msg)
{
    int msg_size = get_length_of_message(msg);
    uint8_t *data = malloc(msg_size);
    int index = 2;

    data[0] = START;
    data[1] = msg->function;

    if (msg_size > MIN_MSG_LENGTH)
    {
        link_t *list_item = msg->content->head;

        while (list_item != NULL)
        {
            data[index++] = SEP;
            index = copy(data, list_item->item, index);
            list_item = list_item->next;
        }
    }

    data[msg_size - 1] = END;

    clear_message(msg);

    return data;
}

int get_length_of_message(MSG_t *msg)
{
    link_t *list_item = msg->content->head;
    int size = 0;

    while (list_item != NULL)
    {
        size += strlen(list_item->item) + 1;
        list_item = list_item->next;
    }

    return size + MIN_MSG_LENGTH;
}

int copy(uint8_t *destination, char *source, int destination_start_index)
{
    int end_index = destination_start_index + strlen(source);

    int i = 0;
    for (i = destination_start_index; i < end_index; i++)
    {
        destination[i] = source[i - destination_start_index];
    }

    return end_index;
}
