
#ifndef _MASTER_COMM_CYCLE_QUEUE_H_
#define _MASTER_COMM_CYCLE_QUEUE_H_

//typedef struct cycle_queue * cycle_queue_t;


typedef struct 
{
    unsigned char * buffer;

    unsigned int front;

    unsigned int element_count;
    unsigned int element_size;
    unsigned int element_count_max;

    int overwrite_when_full;
}cycle_queue;


/// @brief ����һ��ѭ������
/// @param [in] element_size        Ԫ�ش�С
/// @param [in] element_count_max   Ԫ��������
/// @param [in] overwrite_when_full ��֮���Զ�����
/// @return ����ѭ������ʵ��
void cycle_queue_create(cycle_queue * queue, unsigned char * buf, unsigned int element_size, unsigned int element_count_max, int overwrite_when_full);

/// ����һ��ѭ������
void cycle_queue_destroy(cycle_queue * queue);

/// @brief �ڶ���β������һ����Ԫ��
/// @param [in] queue   ���ж���
/// @param [in] element Ԫ��ָ��
/// @return ��ӳɹ�����0������-1
/// @note ��֮�����Ϊȡ����cycle_queue_create��overwrite_when_full����
int cycle_queue_push(cycle_queue * queue, const void * element);

/// ���һ����Ԫ�أ������仺����
unsigned char * cycle_queue_push_empty(cycle_queue * queue);

/// @brief ��ȡ����ͷ������
/// @param [in]  queue    ���ж���
/// @param [out] element  ��Ż�ȡ����Ԫ�����ݵĻ�����
/// @param [in]  pop      ָ���Ƿ�Ӷ�����ɾ��
/// @return �ɹ�����0�����򷵻�-1
int cycle_queue_peek(cycle_queue * queue, void * element, int pop);

/// @brief �Ӷ���ͷ��ȡ��һ��Ԫ��
/// @param [in]  queue    ���ж���
/// @param [out] element  ��Ż�ȡ����Ԫ�����ݵĻ�����
/// @return �ɹ�����0�����򷵻�-1
int cycle_queue_pop(cycle_queue * queue, void * element);

/// �ж϶����Ƿ�Ϊ�գ��շ���1���ǿշ���0
int cycle_queue_is_empty(cycle_queue * queue);

/// �ж϶����Ƿ�Ϊ����������1����������0
int cycle_queue_is_full(cycle_queue * queue);

/// ��ȡ���е�ǰԪ�ظ���
unsigned int cycle_queue_element_count(cycle_queue * queue);

/// ��ȡ�������Ԫ�ظ���
unsigned int cycle_queue_element_count_max(cycle_queue * queue);

#endif
